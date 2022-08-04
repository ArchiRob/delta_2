#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from delta_2.msg import ServoAngles6DoFStamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import State

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

        #broadcast center of manipulator workspace (roughly) as a static tf
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_workspace = TransformStamped()
        tf_workspace.header.frame_id = "stewart_base"
        tf_workspace.header.stamp = rospy.Time.now()
        tf_workspace.child_frame_id = "workspace_center"
        tf_workspace.transform.translation.x = 0.0
        tf_workspace.transform.translation.y = 0.0
        tf_workspace.transform.translation.z = np.sqrt(self.rs**2 - (rb + self.ra - rp)**2)
        tf_workspace.transform.rotation.x = 0.0
        tf_workspace.transform.rotation.y = 0.0
        tf_workspace.transform.rotation.z = 0.0
        tf_workspace.transform.rotation.w = 1.0
        br_static.sendTransform(tf_workspace)

        #init publisher and subscriber
        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions', ServoAngles6DoFStamped, queue_size=1, tcp_nodelay=True) #servo angle publisher
        self.sub_platform_state = rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.callback, tcp_nodelay=True) #target pose subscriber

        #init tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        

    def callback(self, platform_state): #callback calculates servo angles
        #assign positions to vector X
        X = np.asarray([platform_state.pose.position.x, 
                        platform_state.pose.position.y, 
                        platform_state.pose.position.z])

        #get Euler angles from quaternion
        (psi, theta, phi) = euler_from_quaternion([platform_state.pose.orientation.x, 
                                                    platform_state.pose.orientation.y, 
                                                    platform_state.pose.orientation.z, 
                                                    platform_state.pose.orientation.w])

        #initialise empty numpy arrays
        p_w = np.zeros((6,3))
        L_w = np.zeros((6,3))
        Theta = np.zeros(6)
        angle = np.zeros(6)

        #calculate platform rotation matrix wRp
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)
        ctheta = np.cos(theta)
        stheta = np.sin(theta)

        Rpsi = np.asarray([[cpsi, -spsi, 0],
                            [spsi, cpsi, 0],
                            [0, 0, 1]])
    
        Rtheta = np.asarray([[1, 0, 0],
                            [0, ctheta, -stheta],
                            [0, stheta, ctheta]])
      
        Rphi = np.asarray([[cphi, 0, sphi],
                            [0, 1, 0],
                            [-sphi, 0, cphi]])

        wRp = np.matmul(np.matmul(Rpsi, Rtheta), Rphi)

        for i in range(6):
            #calculate distances from platform and base joints
            p_w[i,:] = X + np.matmul(wRp, self.p_p[i,:])
            L_w[i,:] = p_w[i,:] - self.b_w[i,:]
            rl = np.linalg.norm(L_w[i,:])
            L = rl**2 - (self.rs**2 - self.ra**2)

            #convert distances to servo angles
            M = 2 * self.ra * p_w[i,2]
            N = 2 * self.ra * (np.cos(self.beta[i]) * (p_w[i,0] - self.b_w[i,0]) + np.sin(self.beta[i]) * (p_w[i,1] - self.b_w[i,1]))
            disc = L / np.sqrt(M**2 + N**2)

            #check real solution exists -> disc must be in domain of arcsin(), [-1,1]
            if (disc >= 1.0) or (disc <= -1.0):
                Theta[i] = np.nan
            else:
                Theta[i] = np.arcsin(disc) - np.arctan(N / M)

        if not np.any(np.isnan(Theta)):
            #ensure robot wont turn itself inside out (angle between distal linkage and platform cannot exceed 180degrees)
            c30 = np.cos(np.deg2rad(30))
            s30 = np.sin(np.deg2rad(30))
            sTheta = np.sin(Theta)
            cTheta = np.cos(Theta)
            a_w = np.asarray([[self.ra * c30 * cTheta[0], self.ra * s30 * cTheta[0], self.ra * sTheta[0]],
                    [self.ra * c30 * cTheta[1], self.ra * s30 * cTheta[1], self.ra * sTheta[1]],
                    [-self.ra * c30 * cTheta[2], self.ra * s30 * cTheta[2], self.ra * sTheta[2]],
                    [-self.ra * c30 * cTheta[3], self.ra * s30 * cTheta[3], self.ra * sTheta[3]],
                    [0, -self.ra * cTheta[4], self.ra * sTheta[4]],
                    [0, -self.ra * cTheta[5], self.ra * sTheta[5]]])

            n = np.matmul(wRp, np.asarray([0, 0, 1]))
            
            for i in range(6):
                z = - (n[0] * (a_w[i,0] - X[0]) + n[1] * (a_w[i,1] - X[1])) / n[2] + X[2]
                for j in range(6):
                    if a_w[j,2] + 0.01 > z:
                        Theta[i] = np.nan


        #publish if all servo angles have been solved
        if not np.any(np.isnan(Theta)):
            servo_angles = ServoAngles6DoFStamped()
            servo_angles.header.frame_id = "servo"
            servo_angles.header.stamp = platform_state.header.stamp
            servo_angles.Theta1 = np.rad2deg(Theta[0])
            servo_angles.Theta2 = np.rad2deg(Theta[1])
            servo_angles.Theta3 = np.rad2deg(Theta[2])
            servo_angles.Theta4 = np.rad2deg(Theta[3])
            servo_angles.Theta5 = np.rad2deg(Theta[4])
            servo_angles.Theta6 = np.rad2deg(Theta[5])
            self.pub_servo_angles.publish(servo_angles)

            #broadcast transform
            t = TransformStamped()
            t.header.stamp = platform_state.header.stamp
            t.header.frame_id = 'stewart_base'
            t.child_frame_id = 'platform'
            t.transform.translation.x = platform_state.pose.position.x
            t.transform.translation.y = platform_state.pose.position.y
            t.transform.translation.z = platform_state.pose.position.z
            t.transform.rotation.x = platform_state.pose.orientation.x
            t.transform.rotation.y = platform_state.pose.orientation.y
            t.transform.rotation.z = platform_state.pose.orientation.z
            t.transform.rotation.w = platform_state.pose.orientation.w
            self.br.sendTransform(t)
        else:
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS WORKSPACE")

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_kinematics')
    ik = InverseKinematics()
    rospy.spin()