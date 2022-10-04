#!/usr/bin/env python

import rospy
import numpy as np
from delta_2.msg import ServoAnglesStamped
from dynamixel_sdk import *
from dynamic_reconfigure.server import Server
from delta_2.cfg import ServoConfig

def Initialise():
    rospy.loginfo("INITIALISING DYNAMIXELS.......")
    
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(SERIAL_PORT)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(
        portHandler,
        packetHandler,
        ADDR_GOAL_POSITION,
        LEN_GOAL_POSITION)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(
        portHandler,
        packetHandler,
        ADDR_PRESENT_POSITION,
        LEN_PRESENT_POSITION)

    # Open port
    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.loginfo("Failed to open the port")

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.loginfo("Failed to change the baudrate")

    for i in range(NUM_SERVOS):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, 
            int(i+1), 
            ADDR_TORQUE_ENABLE, 
            TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("Dynamixel#%d has been successfully connected" % int(i+1))

        # Add parameter storage for Dynamixel present position
        dxl_addparam_result = groupSyncRead.addParam(i+1)
        if dxl_addparam_result != True:
            rospy.loginfo("[ID:%03d] groupSyncRead addparam failed" % int(i+1))

    return groupSyncWrite, groupSyncRead, portHandler, packetHandler

class ServoController:
    def __init__(self):
        srv = Server(ServoConfig, self.config_callback)
        
        self.pos_pub = rospy.Publisher('/servo_detected/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True) # servo angle publisher

        servo_pos_sub = rospy.Subscriber('/servo_setpoint/positions', ServoAnglesStamped, self.pos_sp_callback, tcp_nodelay=True) #target pose subscriber
        # servo_vel_sub = rospy.Subscriber('/servo_setpoint/velocities', ServoAnglesStamped, self.vel_sp_callback, tcp_nodelay=True) 
        # servo_acc_sub = rospy.Subscriber('/servo_setpoint/accels', ServoAnglesStamped, self.acc_sp_callback, tcp_nodelay=True)

        self.POS_CMD = np.asarray([0, 0, 0, 0, 0, 0])
        # self.DXL_POS_SP = np.asarray([0, 0, 0, 0, 0, 0])
        # self.DXL_VEL_SP = np.asarray([0, 0, 0, 0, 0, 0])
        # self.DXL_ACC_SP = np.asarray([0, 0, 0, 0, 0, 0])
        self.DXL_POS_STAMP = rospy.Time.now()
        self.DXL_VEL_STAMP = rospy.Time.now()
        self.DXL_ACC_STAMP = rospy.Time.now()
        
        rospy.Timer(rospy.Duration(1.0/RATE), self.servo_callback)

    def config_callback(self, config, level): 
        self.settings_changed = True
        self.config = config
        return config
               
    def servo_callback(self, event):
        # Fast Sync Read present position
        dxl_comm_result = groupSyncRead.fastSyncRead()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        dxl_present_position = np.zeros(NUM_SERVOS)
        for i in range(NUM_SERVOS):
            # Check if groupsyncread data of DYNAMIXEL is available
            dxl_getdata_result = groupSyncRead.isAvailable(
                int(i+1),
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                rospy.loginfo("[ID:%03d] groupSyncRead getdata failed" % int(i+1))

            # Get DYNAMIXEL#1 present position value
            dxl_present_position[i] = groupSyncRead.getData(
                int(i+1),
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION)

        #publish measured position
        pos_measured = ServoAnglesStamped()
        pos_measured.header.stamp = rospy.Time.now()
        pos_measured.header.frame_id = "servo"
        for i in range(NUM_SERVOS):
            pos_measured.Theta.append(bits2deg(dxl_present_position[i]))
        self.pos_pub.publish(pos_measured)

        #change servo settings if needed
        if self.settings_changed:
            self.settings_changed = False
            for i in range(NUM_SERVOS):
                #change P gains
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, int(i+1), ADDR_POS_P_GAIN, self.config.P)
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
                #change I gains
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, int(i+1), ADDR_POS_I_GAIN, self.config.I)
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
                #change D gains
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, int(i+1), ADDR_POS_D_GAIN, self.config.D)
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))    
                #change vel_limits
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, int(i+1), ADDR_VEL_LIMIT, self.config.vel_limit)
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))


        for i in range(NUM_SERVOS):

            # Allocate goal position value into byte array
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(deg2bits(self.POS_CMD))),
                DXL_HIBYTE(DXL_LOWORD(deg2bits(self.POS_CMD))),
                DXL_LOBYTE(DXL_HIWORD(deg2bits(self.POS_CMD))),
                DXL_HIBYTE(DXL_HIWORD(deg2bits(self.POS_CMD)))]

            # Add DYNAMIXEL#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(int(i+1), param_goal_position)
            if dxl_addparam_result != True:
                rospy.loginfo("[ID:%03d] groupSyncWrite addparam failed" % int(i+1))

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()



    def pos_sp_callback(self, pos_sub):
        DXL_POS_SP = []

        self.DXL_POS_STAMP = pos_sub.header.stamp

        for i in range(NUM_SERVOS):
            DXL_POS_SP.append(pos_sub.Theta[i])

        self.POS_CMD = DXL_POS_SP

    # def vel_sp_callback(self, vel_sub):
    #     DXL_VEL_SP = []

    #     self.DXL_VEL_STAMP = vel_sub.header.stamp

    #     for i in range(NUM_SERVOS):
    #         DXL_VEL_SP.append(vel_sub.Theta[i])

    #     t = self.DXL_VEL_STAMP - self.DXL_POS_STAMP
    #     t = t.to_sec()

    #     self.POS_CMD += DXL_VEL_SP * t
    #     self.VEL_CMD = DXL_VEL_SP

    # def acc_sp_callback(self, acc_sub):
    #     DXL_ACC_SP = []

    #     self.DXL_ACC_STAMP = acc_sub.header.stamp

    #     for i in range(NUM_SERVOS):
    #         DXL_ACC_SP.append(acc_sub.Theta[i])

    #     t = self.DXL_ACC_STAMP - self.DXL_VEL_STAMP
    #     t = t.to_sec()
        
    #     self.POS_CMD += 0.5 * DXL_ACC_SP * t**2
    #     self.VEL_CMD += DXL_ACC_SP * t
    #     self.ACC_CMD = DXL_ACC_SP

def bits2deg(bits):
    deg = float(bits - 2048) * 0.0878906
    return deg

def deg2bits(deg):
    bits = int(deg / 0.0878906) + 2048
    return bits

def vel2bits(vel):
    bits = int(vel * 0.03816666)
    return bits

    
if __name__ == '__main__':
    # Control table address
    ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_LED_RED            = 65
    ADDR_GOAL_POSITION      = 116
    ADDR_GOAL_VELOCITY      = 104
    ADDR_PRESENT_POSITION   = 132
    ADDR_POS_P_GAIN         = 84
    ADDR_POS_I_GAIN         = 82
    ADDR_POS_D_GAIN         = 80
    ADDR_VEL_LIMIT          = 112

    ADDR_DRIVE_MODE         = 10
    ADDR_OPERATING_MODE     = 11

    # Data Byte Length
    LEN_DRIVE_MODE         = 1
    LEN_OPERATING_MODE     = 1

    LEN_LED_RED             = 1
    LEN_GOAL_VELOCITY       = 4
    LEN_PRESENT_POSITION    = 4
    LEN_GOAL_POSITION       = 4

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    BAUDRATE                    = rospy.get_param('/servo/baud')         # Dynamixel default baudrate : 57600
    SERIAL_PORT                 = rospy.get_param('/servo/port')                # Check which port is being used on your controller
    NUM_SERVOS                  = rospy.get_param('/servo/num')

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque

    RATE = rospy.get_param('/servo/rate')

    rospy.init_node('servo_controller', anonymous=True)

    groupSyncWrite, groupSyncRead, portHandler, packetHandler = Initialise()

    sc = ServoController()

    rospy.spin()

    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    # Disable Dynamixel Torque
    for i in range(NUM_SERVOS):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, int(i+1), ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()