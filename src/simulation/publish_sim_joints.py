#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from delta_2.msg import ServoAngles6DoFStamped

class SimConverter:
    def __init__(self):
        #init publisher and subscriber
        model_name = 'Hex_delta'
        self.pub_servo_angles = rospy.Publisher('/' + model_name + '/position_cmd', Float32MultiArray, queue_size=1, tcp_nodelay=True) #servo angle publisher
        self.sub_platform_state = rospy.Subscriber('/servo_setpoint/positions', ServoAngles6DoFStamped, self.callback, tcp_nodelay=True) #target pose subscriber
        
    def callback(self, platform_state): #callback calculates servo angles
        command = Float32MultiArray()
        command.data.append(np.deg2rad(platform_state.Theta1))
        command.data.append(np.deg2rad(platform_state.Theta2))
        command.data.append(np.deg2rad(platform_state.Theta3))
        command.data.append(np.deg2rad(platform_state.Theta4))
        command.data.append(np.deg2rad(platform_state.Theta5))
        command.data.append(np.deg2rad(platform_state.Theta6))
        self.pub_servo_angles.publish(command)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('publish_sim_joints')
    sc = SimConverter()
    rospy.spin()