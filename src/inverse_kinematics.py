#!/usr/bin/env python
from turtle import home
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
        
        self.rate = rospy.get_param('/servo/rate')

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
        self.Theta = np.zeros(6)
        self.wRp = np.identity(3)
        self.M = np.zeros(6)
        self.N = np.zeros(6)
        self.L_w = np.zeros((6,3))

        self.Q = np.asarray([pos_retracted[0], pos_retracted[1], pos_retracted[2], 0.0, 0.0, 0.0])
        self.Q_dot = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.Q_ddot = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.stamp = rospy.Time.now()

        #broadcast retracted position as tf
        self.br = tf2_ros.TransformBroadcaster()
        # tf_retracted = TransformStamped()
        # tf_retracted.header.frame_id = "stewart_base"
        # tf_retracted.header.stamp = self.stamp
        # tf_retracted.child_frame_id = "platform"
        # tf_retracted.transform.translation.x = self.Q[0]
        # tf_retracted.transform.translation.y = self.Q[1]
        # tf_retracted.transform.translation.z = self.Q[2]
        # tf_retracted.transform.rotation.x = 0.0
        # tf_retracted.transform.rotation.y = 0.0
        # tf_retracted.transform.rotation.z = 0.0
        # tf_retracted.transform.rotation.w = 1.0
        # self.br.sendTransform(tf_retracted)

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
        tf_workspace.header.stamp = self.stamp
        tf_workspace.child_frame_id = "workspace_center"
        tf_workspace.transform.translation = platform_pos_home.pose.position
        tf_workspace.transform.rotation = platform_pos_home.pose.orientation
        br_static.sendTransform(tf_workspace)

        self.home_pos = np.asarray([platform_pos_home.pose.position.x, platform_pos_home.pose.position.y, platform_pos_home.pose.position.z])

        #init publishers and subscribers
        #position kinematics
        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True) #servo angle publisher
        sub_platform_pos = rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.pos_callback, tcp_nodelay=True) #target pose subscriber
        #velocity kinematics
        sub_platform_vel = rospy.Subscriber('/platform_setpoint/velocity', TwistStamped, self.vel_callback, tcp_nodelay=True) #target twist subscriber
        #acceleration kinematics
        sub_platform_vel = rospy.Subscriber('/platform_setpoint/accel', AccelStamped, self.accel_callback, tcp_nodelay=True) #target accel subscriber
        #timer callback
        rospy.Timer(rospy.Duration(1.0/self.rate), self.callback)
        

    def callback(self, event): #callback calculates servo angles
        dt = 1.0 / self.rate

        self.Q_dot += self.Q_ddot * dt

        self.Q += self.Q_dot * dt

        #initialise empty numpy arrays
        p_w = np.zeros((6,3))
        L_w = np.zeros((6,3))

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
                self.Theta[i] = np.nan
            else:
                self.Theta[i] = np.arcsin(disc) - np.arctan(self.N[i] / self.M[i])

        if not np.any(np.isnan(self.Theta)):
            #ensure robot wont turn itself inside out (angle between distal linkage and platform cannot exceed 180degrees)
            c30 = np.cos(np.deg2rad(30))
            s30 = np.sin(np.deg2rad(30))
            sTheta = np.sin(self.Theta)
            cTheta = np.cos(self.Theta)
            a_w = np.asarray([[self.ra * c30 * cTheta[0], self.ra * s30 * cTheta[0], self.ra * sTheta[0]],
                    [self.ra * c30 * cTheta[1], self.ra * s30 * cTheta[1], self.ra * sTheta[1]],
                    [-self.ra * c30 * cTheta[2], self.ra * s30 * cTheta[2], self.ra * sTheta[2]],
                    [-self.ra * c30 * cTheta[3], self.ra * s30 * cTheta[3], self.ra * sTheta[3]],
                    [0, -self.ra * cTheta[4], self.ra * sTheta[4]],
                    [0, -self.ra * cTheta[5], self.ra * sTheta[5]]])

            n = np.matmul(self.wRp, np.asarray([0, 0, 1]))
            
            for i in range(6):
                z = - (n[0] * (a_w[i,0] - X[0]) + n[1] * (a_w[i,1] - X[1])) / n[2] + X[2]
                for j in range(6):
                    if a_w[j,2] + 0.01 > z:
                        self.Theta[i] = np.nan


        #publish if all servo angles have been solved
        if np.any(np.isnan(self.Theta)):
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS MATHEMATICAL WORKSPACE")
        elif np.any(np.abs(X - self.home_pos) > self.translation_limit) or np.any(np.abs(self.Q[3:6]) > np.deg2rad(self.rotation_limit)):
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS DEFINED WORKSPACE")
        else:
            self.servo_angles = ServoAnglesStamped()
            self.servo_angles.header.frame_id = "servo"
            self.servo_angles.header.stamp = rospy.Time.now()
            for i in range(6):
                self.servo_angles.Theta.append(np.rad2deg(self.Theta[i]))

            self.pub_servo_angles.publish(self.servo_angles)  

    def pos_callback(self, platform_pos): 
        #get Euler angles from quaternion
        (theta_0, phi_0, psi_0) = euler_from_quaternion([platform_pos.pose.orientation.x, 
                                                    platform_pos.pose.orientation.y, 
                                                    platform_pos.pose.orientation.z, 
                                                    platform_pos.pose.orientation.w])

        self.Q = np.asarray([platform_pos.pose.position.x, platform_pos.pose.position.y, platform_pos.pose.position.z, 
                            theta_0, phi_0, psi_0])

        self.stamp = platform_pos.header.stamp    

        #broadcast transform
        q = quaternion_from_euler(self.Q[3], self.Q[4], self.Q[5])
        t = TransformStamped()
        t.header.stamp = self.stamp
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

    def vel_callback(self, platform_vel):
        self.Q_dot = np.asarray([platform_vel.twist.linear.x, platform_vel.twist.linear.y, platform_vel.twist.linear.z,
                    platform_vel.twist.angular.z, platform_vel.twist.angular.x, platform_vel.twist.angular.y])

        self.stamp = platform_vel.header.stamp 

    def accel_callback(self, platform_accel):
        self.Q_ddot = np.asarray([platform_accel.accel.linear.x, platform_accel.accel.linear.y, platform_accel.accel.linear.z,
                            0.0, 0.0, 0.0])

        self.stamp = platform_accel.header.stamp 

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_kinematics')
    ik = InverseKinematics()
    rospy.spin()