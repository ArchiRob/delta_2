#!/usr/bin/env python

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

import rospy
import numpy as np
from delta_2.msg import ServoAngles6DoFStamped
from dynamixel_sdk import *
from message_filters import ApproximateTimeSynchronizer, Subscriber

from dynamic_reconfigure.server import Server
from delta_2.cfg import ServoPIDConfig

def Initialise():
    rospy.loginfo("INITIALISING DYNAMIXELS.......")
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupBulkWrite instance
    groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

    # Initialize GroupBulkRead instace for Present Position
    groupBulkRead = GroupBulkRead(portHandler, packetHandler)

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

    for i in range(6):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i+1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("Dynamixel#%d has been successfully connected" % int(i+1))

        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = groupBulkRead.addParam(i+1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_addparam_result != True:
            rospy.loginfo("[ID:%03d] groupBulkRead addparam failed" % int(i+1))

    return groupBulkWrite, groupBulkRead, portHandler, packetHandler

def config_callback(config, level): 
    cfg.P = config.P
    cfg.I = config.I
    cfg.D = config.D
    return config

class cfg:
    P = 0
    I = 0
    D = 0

class ServoController:
    def __init__(self):
        self.DXL_POS_SP = 0.0
        self.DXL_VEL_SP = 0.0
        self.INT_POS_ERR = 0.0
        self.PREV_POS_ERR = 0.0 

        self.pos_pub = rospy.Publisher('/servo_detected/positions', ServoAngles6DoFStamped, queue_size=1, tcp_nodelay=True) # servo angle publisher
        
        pos_sub = Subscriber('/servo_setpoint/positions', ServoAngles6DoFStamped) #target angle subscriber
        vel_sub = Subscriber('/servo_setpoint/velocities', ServoAngles6DoFStamped) #current limit subscriber

        self.PID = []
        for i in range(6):
            self.PID.append(pid())

        print(self.PID)

        ts = ApproximateTimeSynchronizer([pos_sub, vel_sub], queue_size=1, slop=0.05)
        ts.registerCallback(self.update_sp_callback)
        
        rospy.Timer(rospy.Duration(1.0/RATE), self.servo_callback)

    def servo_callback(self, event):

        # Bulkread present position
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
        dxl_present_position = np.zeros(6)

        # Get present position value
        for i in range(6):
            # Check if groupbulkread data of Dynamixel is available
            dxl_getdata_result = groupBulkRead.isAvailable(i+1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                rospy.loginfo("[ID:%03d] groupBulkRead getdata failed" % int(i+1))
            
            dxl_present_position[i] = groupBulkRead.getData(i+1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        pos_measured = ServoAngles6DoFStamped()
        pos_measured.header.stamp = rospy.Time.now()
        pos_measured.header.frame_id = "servo"
        pos_measured.Theta1 = bits2deg(dxl_present_position[0])
        pos_measured.Theta2 = bits2deg(dxl_present_position[1])
        pos_measured.Theta3 = bits2deg(dxl_present_position[2])
        pos_measured.Theta4 = bits2deg(dxl_present_position[3])
        pos_measured.Theta5 = bits2deg(dxl_present_position[4])
        pos_measured.Theta6 = bits2deg(dxl_present_position[5])
        self.pos_pub.publish(pos_measured)


        for i in range(6):
            DXL_VEL = map(self.PID.cmd_vel(self.DXL_POS_SP[i], self.DXL_VEL_SP[i], dxl_present_position[i]), self.PID)
            DXL_VEL = vel2bits(DXL_VEL)
            
            # Allocate goal position value into byte array
            param_goal_vel = [DXL_LOBYTE(DXL_LOWORD(DXL_VEL[i])), DXL_HIBYTE(DXL_LOWORD(DXL_VEL[i])), DXL_LOBYTE(DXL_HIWORD(DXL_VEL[i])), DXL_HIBYTE(DXL_HIWORD(DXL_VEL[i]))]
        
            # Add Dynamixel goal position value to the Bulkwrite parameter storage
            dxl_addparam_result = groupBulkWrite.addParam(i+1, ADDR_PRO_GOAL_VELOCITY, LEN_PRO_GOAL_VELOCITY, param_goal_vel)
            if dxl_addparam_result != True:
                rospy.loginfo("[ID:%03d] groupBulkWrite addparam position failed" % int(i+1))
            
        # Bulkwrite goal velocity
        dxl_comm_result = groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        groupBulkWrite.clearParam()

    def update_sp_callback(self, pos_sub, vel_sub):
        self.DXL_POS_SP = []
        self.DXL_VEL_SP = []

        self.DXL_POS_SP[0] = pos_sub.Theta1
        self.DXL_POS_SP[1] = pos_sub.Theta2
        self.DXL_POS_SP[2] = pos_sub.Theta3
        self.DXL_POS_SP[3] = pos_sub.Theta4
        self.DXL_POS_SP[4] = pos_sub.Theta5
        self.DXL_POS_SP[5] = pos_sub.Theta6

        self.DXL_VEL_SP[0] = vel_sub.Theta1
        self.DXL_VEL_SP[1] = vel_sub.Theta2
        self.DXL_VEL_SP[2] = vel_sub.Theta3
        self.DXL_VEL_SP[3] = vel_sub.Theta4
        self.DXL_VEL_SP[4] = vel_sub.Theta5
        self.DXL_VEL_SP[5] = vel_sub.Theta6

def bits2deg(bits):
    deg = float(bits - 2048) / 4096.0 * 360.0
    return deg

def vel2bits(vel):
    bits = int(vel * 0.229 * 60 / 360)
    return bits


class pid:
    def __init__(self):
        self.int_err = 0.0
        self.err = 0.0

    def cmd_vel(self, pos_sp, vel_sp, pos_measured):
        
        err = pos_sp - pos_measured

        self.int_err += err / RATE

        deriv_err = (err - self.err) * RATE

        vel_cmd = cfg.P * err + cfg.I * self.int_err + cfg.D * deriv_err + vel_sp 

        self.err = err

        return vel_cmd
    
if __name__ == '__main__':
    # Control table address
    ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_PRO_LED_RED            = 65
    ADDR_PRO_GOAL_VELOCITY      = 104
    ADDR_PRO_PRESENT_POSITION   = 132

    ADDR_PRO_DRIVE_MODE         = 10
    ADDR_PRO_OPERATING_MODE     = 11

    # Data Byte Length
    LEN_PRO_DRIVE_MODE         = 1
    LEN_PRO_OPERATING_MODE     = 1

    LEN_PRO_LED_RED             = 1
    LEN_PRO_GOAL_VELOCITY       = 4
    LEN_PRO_PRESENT_POSITION    = 4

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    BAUDRATE                    = 4000000          # Dynamixel default baudrate : 57600
    DEVICENAME                  = "/dev/ttyUSB0"                # Check which port is being used on your controller

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque

    RATE = 100

    rospy.init_node('servo_controller', anonymous=True)
    srv = Server(ServoPIDConfig, config_callback)

    groupBulkWrite, groupBulkRead, portHandler, packetHandler = Initialise()

    sc = ServoController()

    rospy.spin()

    # Clear bulkread parameter storage
    groupBulkRead.clearParam()

    # Disable Dynamixel Torque
    for i in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i+1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()