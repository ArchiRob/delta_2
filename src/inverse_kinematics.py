#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Vector3Stamped
from delta_2.msg import ServoAnglesStamped
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

        self.Theta = np.zeros(6)
        self.Theta_dot = np.zeros(6)
        self.Theta_ddot = np.zeros(6)
        self.wRp = np.identity(3)
        self.M = np.zeros(6)
        self.N = np.zeros(6)
        self.L_w = np.zeros((6,3))

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

        #kinematics for initial platform pose
        platform_pos_init = PoseStamped()
        platform_pos_init.pose.position.x = 0.0
        platform_pos_init.pose.position.y = 0.0
        platform_pos_init.pose.position.z = np.sqrt(self.rs**2 - (rb + self.ra - rp)**2)
        platform_pos_init.pose.orientation.x = 0.0
        platform_pos_init.pose.orientation.y = 0.0
        platform_pos_init.pose.orientation.z = 0.0
        platform_pos_init.pose.orientation.w = 1.0

        #generate initial values
        self.nopub = True
        self.pos_callback(platform_pos_init)
        self.nopub = False

        #broadcast center of manipulator workspace (roughly) as a static tf
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_workspace = TransformStamped()
        tf_workspace.header.frame_id = "stewart_base"
        tf_workspace.header.stamp = rospy.Time.now()
        tf_workspace.child_frame_id = "workspace_center"
        tf_workspace.transform.translation = platform_pos_init.pose.position
        tf_workspace.transform.rotation = platform_pos_init.pose.orientation
        br_static.sendTransform(tf_workspace)

        pos_retracted = rospy.get_param('/retracted_position')

        #broadcast retracted position (roughly) as a static tf#
        self.br = tf2_ros.TransformBroadcaster()
        tf_retracted = TransformStamped()
        tf_retracted.header.frame_id = "stewart_base"
        tf_retracted.header.stamp = rospy.Time.now()
        tf_retracted.child_frame_id = "platform"
        tf_retracted.transform.translation.x = pos_retracted[0]
        tf_retracted.transform.translation.y = pos_retracted[1]
        tf_retracted.transform.translation.z = pos_retracted[2]
        tf_retracted.transform.rotation = platform_pos_init.pose.orientation
        self.br.sendTransform(tf_retracted)

        #init publishers and subscribers
        #position kinematics
        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True) #servo angle publisher
        sub_platform_pos = rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.pos_callback, tcp_nodelay=True) #target pose subscriber
        #velocity kinematics
        self.pub_servo_velocities = rospy.Publisher('/servo_setpoint/velocities', ServoAnglesStamped, queue_size=1) #servo velocity publisher
        sub_platform_vel = rospy.Subscriber('/platform_setpoint/velocity', TwistStamped, self.vel_callback, tcp_nodelay=True) #target twist subscriber
        #acceleration kinematics
        self.pub_servo_accels = rospy.Publisher('/servo_setpoint/accels', ServoAnglesStamped, queue_size=1) #servo accel publisher
        sub_platform_vel = rospy.Subscriber('/platform_setpoint/accel', Vector3Stamped, self.accel_callback, tcp_nodelay=True) #target accel subscriber

        #init tf broadcaster
        
        

    def pos_callback(self, platform_pos): #callback calculates servo angles
        #assign positions to vector X
        X = np.asarray([platform_pos.pose.position.x, 
                        platform_pos.pose.position.y, 
                        platform_pos.pose.position.z])

        #get Euler angles from quaternion
        (self.psi, self.theta, self.phi) = euler_from_quaternion([platform_pos.pose.orientation.x, 
                                                    platform_pos.pose.orientation.y, 
                                                    platform_pos.pose.orientation.z, 
                                                    platform_pos.pose.orientation.w])

        #initialise empty numpy arrays
        p_w = np.zeros((6,3))
        L_w = np.zeros((6,3))

        #calculate platform rotation matrix wRp
        cphi = np.cos(self.phi)
        sphi = np.sin(self.phi)
        cpsi = np.cos(self.psi)
        spsi = np.sin(self.psi)
        ctheta = np.cos(self.theta)
        stheta = np.sin(self.theta)

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
        if not (np.any(np.isnan(self.Theta))):
            self.servo_angles = ServoAnglesStamped()
            self.servo_angles.header.frame_id = "servo"
            self.servo_angles.header.stamp = platform_pos.header.stamp
            for i in range(6):
                self.servo_angles.Theta.append(np.rad2deg(self.Theta[i]))
            if not self.nopub:
                self.pub_servo_angles.publish(self.servo_angles)

            #broadcast transform
            t = TransformStamped()
            t.header.stamp = platform_pos.header.stamp
            t.header.frame_id = 'stewart_base'
            t.child_frame_id = 'platform'
            t.transform.translation = platform_pos.pose.position
            t.transform.rotation = platform_pos.pose.orientation
            if not self.nopub:
                self.br.sendTransform(t)
        else:
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS WORKSPACE")

    def vel_callback(self, platform_vel):
        #initialise empty numpy arrays
        n = np.zeros((6,3))
        p = np.zeros((6,3))
        J1_inv = np.zeros((6,6))
        J2_inv = np.identity(6)

        #rotate to avoid singularity when platform is level
        theta = self.theta - np.pi/2
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        cpsi = np.cos(self.psi)
        spsi = np.sin(self.psi)

        #assign velocities to vector q_dot
        #angular velocities are assigned in a weird order to represent a frame rotation of -pi/2 about the x axis. This allows us to avoid a singularity.
        Q_dot = np.asarray([platform_vel.twist.linear.x, platform_vel.twist.linear.y, platform_vel.twist.linear.z,
                            platform_vel.twist.angular.x, platform_vel.twist.angular.y, platform_vel.twist.angular.z])
        
        for i in range(6):
            #this is the new stuff to calculate velocities
            n[i,:] = np.divide(self.L_w[i,:], np.linalg.norm(self.L_w[i,:]))
            w = np.cross(self.p_p[i,:], n[i,:])
            p[i,:] = np.matmul(self.wRp, w)
        
        J1_inv = np.column_stack((n, p))

        #angular velocities
        omega = np.asarray([[0, cpsi, spsi * stheta],
                [0, spsi, -cpsi * stheta],
                [1, 0, ctheta]])    

        J2_inv[3:6, 3:6] = omega

        L_w_dot = np.matmul(np.matmul(J1_inv, J2_inv), Q_dot)

        for i in range(6):
            self.Theta_dot[i] = L_w_dot[i] / (self.M[i] * np.cos(self.Theta[i]) - self.N[i] * np.sin(self.Theta[i]))

        #publish
        self.servo_velocities = ServoAnglesStamped()
        self.servo_velocities.header.frame_id = "servo"
        self.servo_velocities.header.stamp = platform_vel.header.stamp
        for i in range(6):
            self.servo_velocities.Theta.append(np.rad2deg(self.Theta_dot[i]))
        self.pub_servo_velocities.publish(self.servo_velocities)

        # #increment Theta? todo:see if this helps when velocity sp frequency is high
        # self.Theta += self.Theta_dot * (self.servo_velocities.header.stamp - self.servo_angles.header.stamp).to_sec()

    def accel_callback(self, platform_accel):
        #publish
        self.servo_accels = ServoAnglesStamped()
        self.servo_accels.header.frame_id = "servo"
        self.servo_accels.header.stamp = platform_accel.header.stamp
        for i in range(6):
            self.servo_accels.Theta.append(np.rad2deg(self.Theta_ddot[i]))
        self.pub_servo_accels.publish(self.servo_accels)

        # #increment Theta? todo:see if this helps when acc sp frequency is high
        # self.Theta += 0.5 * self.Theta_ddot * ((self.servo_accel.header.stamp - self.servo_angles.header.stamp).to_sec())**2
        # self.Theta_dot += self.Theta_ddot * (self.servo_accel.header.stamp - self.servo_angles.header.stamp).to_sec()

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_kinematics')
    ik = InverseKinematics()
    rospy.spin()