#!/usr/bin/env python

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
class InverseKinematics:
    def __init__(self):
        #get geometry values form parameter server
        rb = rospy.get_param('/base_radius')
        rp = rospy.get_param('/platform_radius')
        sb = rospy.get_param('/base_joint_spacing')
        sp = rospy.get_param('/platform_joint_spacing')
        self.ra = rospy.get_param('/proximal_link_length')
        self.rs = rospy.get_param('/distal_link_length')

        pos_retracted = rospy.get_param('/retracted_position')
        home_offset = rospy.get_param('/home_offset')
        self.translation_limit = rospy.get_param('/translation_limit')
        self.rotation_limit = rospy.get_param('/rotation_limit')
        self.solve = False
    
        #angles of servo motors about centre of base
        self.beta = np.asarray([np.deg2rad(30), np.deg2rad(30), np.deg2rad(150), np.deg2rad(150), np.deg2rad(270), np.deg2rad(270)])

        #calculate coordinates of wrist and shoulder joints in their respective body coordinates
        c30 = np.cos(np.deg2rad(30))
        s30 = np.sin(np.deg2rad(30))

        #platform joints in platform coordinates
        self.p_p = np.asarray([[rp * c30 + sp * s30, rp * s30 - sp * c30, 0],
            [rp * c30 - sp * s30, rp * s30 + sp * c30, 0],
            [-rp * c30 + sp * s30, rp * s30 + sp * c30, 0],
            [-rp * c30 - sp * s30, rp * s30 - sp * c30, 0],
            [-sp, -rp, 0],
            [sp, -rp, 0]])

        #base joints in base/world coordinates
        self.b_w = np.asarray([[rb * c30 + sb * s30, rb * s30 - sb * c30, 0],
            [rb * c30 - sb * s30, rb * s30 + sb * c30, 0],
            [-rb * c30 + sb * s30, rb * s30 + sb * c30, 0],
            [-rb * c30 - sb * s30, rb * s30 - sb * c30, 0],
            [-sb, -rb, 0],
            [sb, -rb, 0]])

        #generate initial values
        self.wRp = np.identity(3)
        self.M = np.ones(6)
        self.N = np.ones(6)
        self.L_w = np.ones((6,3))

        self.Q = np.asarray([pos_retracted[0], pos_retracted[1], pos_retracted[2], 0.0, 0.0, 0.0])
        self.Q_dot = np.zeros(6)
        self.Q_ddot = np.zeros(6)
        
        self.Theta = np.zeros(6)
        self.Theta_dot = np.zeros(6)
        self.Theta_ddot = np.zeros(6)

        self.br = tf2_ros.TransformBroadcaster()

        #determine platform home position and broadcast tf
        platform_pos_home = PoseStamped()
        platform_pos_home.pose.position.x = home_offset[0]
        platform_pos_home.pose.position.y = home_offset[1]
        platform_pos_home.pose.position.z = np.sqrt(self.rs**2 - (rb + self.ra - rp)**2) + home_offset[2]
        platform_pos_home.pose.orientation.x = 0.0
        platform_pos_home.pose.orientation.y = 0.0
        platform_pos_home.pose.orientation.z = 0.0
        platform_pos_home.pose.orientation.w = 1.0
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_workspace = TransformStamped()
        tf_workspace.header.frame_id = "stewart_base"
        tf_workspace.header.stamp = rospy.Time.now()
        tf_workspace.child_frame_id = "workspace_center"
        tf_workspace.transform.translation = platform_pos_home.pose.position
        tf_workspace.transform.rotation = platform_pos_home.pose.orientation
        br_static.sendTransform(tf_workspace)

        self.home_pos = np.asarray([platform_pos_home.pose.position.x, platform_pos_home.pose.position.y, platform_pos_home.pose.position.z])

        #init publishers and subscribers
        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)

        #position kinematics
        sub_platform_pos = rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.pos_callback, tcp_nodelay=True) #target pose subscriber
        
    def pos_callback(self, platform_pos): 
        #get Euler angles from quaternion
        (theta, phi, psi) = euler_from_quaternion([platform_pos.pose.orientation.x, 
                                                    platform_pos.pose.orientation.y, 
                                                    platform_pos.pose.orientation.z, 
                                                    platform_pos.pose.orientation.w])

        self.Q = np.asarray([platform_pos.pose.position.x, platform_pos.pose.position.y, platform_pos.pose.position.z, 
                            theta, phi, psi]) 

        #broadcast transform
        q = quaternion_from_euler(self.Q[3], self.Q[4], self.Q[5])
        t = TransformStamped()
        t.header.stamp = platform_pos.header.stamp
        t.header.frame_id = 'stewart_base'
        t.child_frame_id = 'platform'
        t.transform.translation.x = self.Q[0]
        t.transform.translation.y = self.Q[1]
        t.transform.translation.z = self.Q[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t) 

        #initialise empty numpy arrays
        p_w = np.zeros((6,3))
        L_w = np.zeros((6,3))
        Theta = np.zeros(6)

        #calculate platform rotation matrix wRp
        cphi = np.cos(self.Q[4])
        sphi = np.sin(self.Q[4])
        cpsi = np.cos(self.Q[5])
        spsi = np.sin(self.Q[5])
        ctheta = np.cos(self.Q[3])
        stheta = np.sin(self.Q[3])

        Rpsi = np.asarray([[cpsi, -spsi, 0],
                            [spsi, cpsi, 0],
                            [0, 0, 1]])
    
        Rtheta = np.asarray([[1, 0, 0],
                            [0, ctheta, -stheta],
                            [0, stheta, ctheta]])
      
        Rphi = np.asarray([[cphi, 0, sphi],
                            [0, 1, 0],
                            [-sphi, 0, cphi]])

        self.wRp = np.matmul(np.matmul(Rpsi, Rtheta), Rphi)

        X = self.Q[0:3]

        for i in range(6):
            #calculate distances from platform and base joints
            p_w[i,:] = X + np.matmul(self.wRp, self.p_p[i,:])
            self.L_w[i,:] = p_w[i,:] - self.b_w[i,:]
            rl = np.linalg.norm(self.L_w[i,:])
            L = rl**2 - (self.rs**2 - self.ra**2)

            #convert distances to servo angles
            self.M[i] = 2 * self.ra * p_w[i,2]
            self.N[i] = 2 * self.ra * (np.cos(self.beta[i]) * (p_w[i,0] - self.b_w[i,0]) + np.sin(self.beta[i]) * (p_w[i,1] - self.b_w[i,1]))
            disc = L / np.sqrt(self.M[i]**2 + self.N[i]**2)

            #check real solution exists -> disc must be in domain of arcsin(), [-1,1]
            if (disc >= 1.0) or (disc <= -1.0):
                Theta[i] = np.nan
            else:
                Theta[i] = np.arcsin(disc) - np.arctan(self.N[i] / self.M[i])

        #publish if all servo angles have been solved and angles are within defined limits
        if np.any(np.isnan(Theta)):
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS MATHEMATICAL WORKSPACE")
            self.solve = False
        elif np.any(np.abs(X - self.home_pos) > self.translation_limit) or np.any(np.abs(self.Q[3:6]) > np.deg2rad(self.rotation_limit)):
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS DEFINED WORKSPACE")
            self.solve = False
        else:
            self.Theta = Theta
            self.solve = True
            servo_angles = ServoAnglesStamped()
            servo_angles.header.frame_id = "servo"
            servo_angles.header.stamp = platform_pos.header.stamp
            for i in range(6):
                servo_angles.Theta.append(np.rad2deg(self.Theta[i]))

            self.pub_servo_angles.publish(servo_angles)  

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_kinematics')
    ik = InverseKinematics()
    rospy.spin()