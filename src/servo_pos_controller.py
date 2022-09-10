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

    for i in range(6):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, 
            i+1, 
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
            print("[ID:%03d] groupSyncRead addparam failed" % int(i+1))

    return groupSyncWrite, groupSyncRead, portHandler, packetHandler

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
        self.pos_pub = rospy.Publisher('/servo_detected/positions', ServoAngles6DoFStamped, queue_size=1, tcp_nodelay=True) # servo angle publisher
        
        pos_sub = Subscriber('/servo_setpoint/positions', ServoAngles6DoFStamped) #target angle subscriber
        vel_sub = Subscriber('/servo_setpoint/velocities', ServoAngles6DoFStamped) #current limit subscriber

        self.DXL_POS_SP = np.asarray([0, 0, 0, 0, 0, 0])

        ts = ApproximateTimeSynchronizer([pos_sub, vel_sub], queue_size=1, slop=0.05)
        ts.registerCallback(self.update_sp_callback)
        
        rospy.Timer(rospy.Duration(1.0/RATE), self.servo_callback)
        
        

    def servo_callback(self, event):
        # Fast Sync Read present position
        dxl_comm_result = groupSyncRead.fastSyncRead()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        dxl_present_position = np.zeros(6)
        for i in range(6):
            # Check if groupsyncread data of DYNAMIXEL is available
            dxl_getdata_result = groupSyncRead.isAvailable(
                i+1,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % int(i+1))

            # Get DYNAMIXEL#1 present position value
            dxl_present_position[i] = groupSyncRead.getData(
                i+1,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION)

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
            POS_SP = deg2bits(self.DXL_POS_SP[i])
            # Allocate goal position value into byte array
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(POS_SP)),
                DXL_HIBYTE(DXL_LOWORD(POS_SP)),
                DXL_LOBYTE(DXL_HIWORD(POS_SP)),
                DXL_HIBYTE(DXL_HIWORD(POS_SP))]

            # Add DYNAMIXEL#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(i+1, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % int(i+1))

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()



    def update_sp_callback(self, pos_sub, vel_sub):
        self.DXL_POS_SP = np.zeros(6)
        self.DXL_VEL_SP = np.zeros(6)

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
    BAUDRATE                    = 4000000          # Dynamixel default baudrate : 57600
    SERIAL_PORT                 = "/dev/ttyUSB0"                # Check which port is being used on your controller

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque

    RATE = 500

    rospy.init_node('servo_controller', anonymous=True)
    srv = Server(ServoPIDConfig, config_callback)

    groupSyncWrite, groupSyncRead, portHandler, packetHandler = Initialise()

    sc = ServoController()

    rospy.spin()

    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    # Disable Dynamixel Torque
    for i in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i+1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()