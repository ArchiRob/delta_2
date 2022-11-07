#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout
from delta_2.msg import ServoAnglesStamped

class SimConverter:
    def __init__(self):
        self.rate = rospy.get_param('/servo/rate')

        self.pos_sp = np.zeros(rospy.get_param('/servo/num'))
        self.vel_sp = np.zeros(rospy.get_param('/servo/num'))
        self.acc_sp = np.zeros(rospy.get_param('/servo/num'))
        self.stamp_latest = rospy.Time.now()
        self.vel_stamp_last = rospy.Time.now()
        self.acc_stamp_last = rospy.Time.now()

        #init publisher and subscriber
        model_name = rospy.get_param('model_name')
        self.pub_servo_angles = rospy.Publisher('/' + model_name + '/position_cmd', Float32MultiArray, queue_size=1, tcp_nodelay=True)
        sub_platform_state = rospy.Subscriber('/servo_setpoint/positions', ServoAnglesStamped, self.pos_callback, tcp_nodelay=True)
        sub_platform_vel = rospy.Subscriber('/servo_setpoint/velocities', ServoAnglesStamped, self.vel_callback, tcp_nodelay=True) 
        sub_platform_acc = rospy.Subscriber('/servo_setpoint/accels', ServoAnglesStamped, self.acc_callback, tcp_nodelay=True) 
        rospy.Timer(rospy.Duration(1.0/self.rate), self.servo_callback)

    def servo_callback(self, event):
        dt = 1.0/self.rate
        self.pos_sp += self.vel_sp * dt + 0.5 * self.acc_sp * dt**2
        # print(self.pos_sp)
        angles = np.deg2rad(self.pos_sp).tolist()
        self.pub_servo_angles.publish(Float32MultiArray(MultiArrayLayout(), angles))
    
    def pos_callback(self, platform_state): #callback calculates servo angles
        self.pos_sp = np.asarray(platform_state.Theta)
        self.stamp_latest = platform_state.header.stamp

    def vel_callback(self, platform_vel): #callback calculates servo angles
        self.vel_sp = np.asarray(platform_vel.Theta)
        dt = rospy.Time.to_sec(platform_vel.header.stamp) - rospy.Time.to_sec(self.vel_stamp_last)
        self.vel_stamp_last = platform_vel.header.stamp
        self.pos_sp += self.vel_sp * dt
        self.stamp_latest = platform_vel.header.stamp

    def acc_callback(self, platform_acc):
        self.acc_sp = np.asarray(platform_acc.Theta)
        dt = rospy.Time.to_sec(platform_acc.header.stamp) - rospy.Time.to_sec(self.acc_stamp_last)
        self.acc_stamp_last = platform_acc.header.stamp
        self.vel_sp = self.acc_sp * dt
        self.stamp_latest = platform_acc.header.stamp

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('publish_sim_joints')
    sc = SimConverter()
    rospy.spin()