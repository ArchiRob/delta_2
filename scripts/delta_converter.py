#!/usr/bin/env python3

#-------------------------------------------------------------------------------------------------
# INVERSE KINEMATICS
# This script computes the inverse kinematics solution for the stewart platform. Basically
# it takes position/velocity/acceleration setpoints for the end-effector and works out what
# the motors need to do. A nice perk of using a parallel manipulator is that these equations
# can be solved directly (so no faffing about with MoveIt! needed)
# The callbacks all operate independently so the publishing rates depend on the rate of the input
# setpoints and are not dependent on eachother.
#-------------------------------------------------------------------------------------------------

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, AccelStamped
from delta_2.msg import ServoAnglesStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#code to determine servo positions from end effector pose setpoints
class deltaConverter:
    def __init__(self):      
        sp = rospy.get_param('/platform_joint_spacing')
        sb = sp #we ignore base joint spacing so linkages are always parallel and overwrite it to prevent issues
        rospy.set_param('/base_joint_spacing', sb)

        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions3dof', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)
        self.pub_servo_velocities = rospy.Publisher('/servo_setpoint/velocities3dof', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)
        self.pub_servo_accels = rospy.Publisher('/servo_setpoint/accels3dof', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)
        sub_platform_pos = rospy.Subscriber('/servo_setpoint/positions6dof', ServoAnglesStamped, self.pos_callback, tcp_nodelay=True) #target pose subscriber
        sub_platform_vel = rospy.Subscriber('/servo_setpoint/velocities6dof', ServoAnglesStamped, self.vel_callback, tcp_nodelay=True) #target twist subscriber
        sub_platform_vel = rospy.Subscriber('/servo_setpoint/accels6dof', ServoAnglesStamped, self.accel_callback, tcp_nodelay=True) #target accel subscriber

    def pos_callback(self, pos_msg6dof):
        pos_msg3dof = ServoAnglesStamped()
        pos_msg3dof.header = pos_msg6dof.header
        pos_msg3dof.Theta.append(pos_msg6dof.Theta[1])
        pos_msg3dof.Theta.append(pos_msg6dof.Theta[3])
        pos_msg3dof.Theta.append(pos_msg6dof.Theta[5])
        self.pub_servo_angles.publish(pos_msg3dof)

    def vel_callback(self, vel_msg6dof):
        vel_msg3dof = ServoAnglesStamped()
        vel_msg3dof.header = vel_msg6dof.header
        vel_msg3dof.Theta.append(vel_msg6dof.Theta[1])
        vel_msg3dof.Theta.append(vel_msg6dof.Theta[3])
        vel_msg3dof.Theta.append(vel_msg6dof.Theta[5])
        self.pub_servo_velocities.publish(vel_msg3dof)

    def accel_callback(self, acc_msg6dof):
        acc_msg3dof = ServoAnglesStamped()
        acc_msg3dof.header = acc_msg6dof.header
        acc_msg3dof.Theta.append(acc_msg6dof.Theta[1])
        acc_msg3dof.Theta.append(acc_msg6dof.Theta[3])
        acc_msg3dof.Theta.append(acc_msg6dof.Theta[5])
        self.pub_servo_accels.publish(acc_msg3dof)


if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_converter')
    dC = deltaConverter()
    rospy.spin()