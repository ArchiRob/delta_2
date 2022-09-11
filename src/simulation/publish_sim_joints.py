#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from delta_2.msg import ServoAnglesStamped

class SimConverter:
    def __init__(self):
        #init publisher and subscriber
        model_name = rospy.get_param('model_name')
        self.pub_servo_angles = rospy.Publisher('/' + model_name + '/position_cmd', Float32MultiArray, queue_size=1, tcp_nodelay=True) #servo angle publisher
        self.sub_platform_state = rospy.Subscriber('/servo_setpoint/positions', ServoAnglesStamped, self.pos_callback, tcp_nodelay=True) #target pose subscriber

        self.pub_servo_vels = rospy.Publisher('/' + model_name + '/velocity_cmd', Float32MultiArray, queue_size=1, tcp_nodelay=True) #servo angle publisher
        self.sub_platform_vel = rospy.Subscriber('/servo_setpoint/velocities', ServoAnglesStamped, self.vel_callback, tcp_nodelay=True) #target pose subscriber
        
    def pos_callback(self, platform_state): #callback calculates servo angles
        command = Float32MultiArray()
        command.data.append(np.deg2rad(platform_state.Theta[0]))
        command.data.append(np.deg2rad(platform_state.Theta[1]))
        command.data.append(np.deg2rad(platform_state.Theta[2]))
        command.data.append(np.deg2rad(platform_state.Theta[3]))
        command.data.append(np.deg2rad(platform_state.Theta[4]))
        command.data.append(np.deg2rad(platform_state.Theta[5]))
        self.pub_servo_angles.publish(command)

    def vel_callback(self, platform_state): #callback calculates servo angles
        command = Float32MultiArray()
        command.data.append(np.deg2rad(platform_state.Theta[0]))
        command.data.append(np.deg2rad(platform_state.Theta[1]))
        command.data.append(np.deg2rad(platform_state.Theta[2]))
        command.data.append(np.deg2rad(platform_state.Theta[3]))
        command.data.append(np.deg2rad(platform_state.Theta[4]))
        command.data.append(np.deg2rad(platform_state.Theta[5]))
        self.pub_servo_vels.publish(command)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('publish_sim_joints')
    sc = SimConverter()
    rospy.spin()