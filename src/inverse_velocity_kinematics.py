#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from delta_2.msg import ServoAnglesStamped
from tf.transformations import euler_from_quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber

#code to determine servo velocities from end effector velocity setpoints
#todo: there is a singularity when the platform is level (phi and theta are aligned), maybe quaternions can fix this?

class InverseDynamics:
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

        #init publisher and subscriber
        self.pub_servo_velocities = rospy.Publisher('/servo_setpoint/velocities', ServoAnglesStamped, queue_size=1) #servo velocity publisher
        sub_platform_vel = Subscriber('/platform_setpoint/velocity', TwistStamped) #target twist subscriber
        sub_platform_pose = Subscriber('/platform_setpoint/pose', PoseStamped) #target pose subscriber
        sub_servo_angles = Subscriber('/servo_setpoint/positions', ServoAnglesStamped) #servo angle subscriber

        ts = ApproximateTimeSynchronizer([sub_platform_vel, sub_platform_pose, sub_servo_angles], queue_size=1, slop=0.05)
        ts.registerCallback(self.vel_callback)

    def vel_callback(self, platform_vel, platform_pose, servo_angles):
        #initialise empty numpy arrays
        p_w = np.zeros((6,3))
        L_w = np.zeros((6,3))
        n = np.zeros((6,3))
        p = np.zeros((6,3))
        J1_inv = np.zeros((6,6))
        J2_inv = np.identity(6)
        Theta_dot = np.zeros(6)
        M = np.zeros(6)
        N = np.zeros(6)

        #put calculated servo angles in array
        Theta = np.asarray([np.deg2rad(servo_angles.Theta[0]), 
                                np.deg2rad(servo_angles.Theta[1]),
                                np.deg2rad(servo_angles.Theta[2]), 
                                np.deg2rad(servo_angles.Theta[3]), 
                                np.deg2rad(servo_angles.Theta[4]), 
                                np.deg2rad(servo_angles.Theta[5])])

        #get Euler angles from quaternion
        (psi, theta, phi) = euler_from_quaternion([platform_pose.pose.orientation.x, 
                                                    platform_pose.pose.orientation.y, 
                                                    platform_pose.pose.orientation.z, 
                                                    platform_pose.pose.orientation.w])

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

        X = np.asarray([platform_pose.pose.position.x, 
                        platform_pose.pose.position.y, 
                        platform_pose.pose.position.z])

        for i in range(6):
            #calculate distances from platform and base joints
            p_w[i,:] = X + np.matmul(wRp, self.p_p[i,:])
            L_w[i,:] = p_w[i,:] - self.b_w[i,:]
            M[i] = 2 * self.ra * p_w[i,2]
            N[i] = 2 * self.ra * (np.cos(self.beta[i]) * (p_w[i,0] - self.b_w[i,0]) + np.sin(self.beta[i]) * (p_w[i,1] - self.b_w[i,1]))

        #rotate to avoid singularity when platform is level
        theta = theta - np.pi/2
        ctheta = np.cos(theta)
        stheta = np.sin(theta)

        #assign velocities to vector q_dot
        #angular velocities are assigned in a weird order to represent a frame rotation of -pi/2 about the x axis. This allows us to avoid a singularity.
        Q_dot = np.asarray([platform_vel.twist.linear.x, platform_vel.twist.linear.y, platform_vel.twist.linear.z,
                            platform_vel.twist.angular.x, platform_vel.twist.angular.y, platform_vel.twist.angular.z])
        
        for i in range(6):
            #this is the new stuff to calculate velocities
            n[i,:] = np.divide(L_w[i,:], np.linalg.norm(L_w[i,:]))
            w = np.cross(self.p_p[i,:], n[i,:])
            p[i,:] = np.matmul(wRp, w)
        
        J1_inv = np.column_stack((n, p))

        #angular velocities
        omega = np.asarray([[0, cpsi, spsi * stheta],
                [0, spsi, -cpsi * stheta],
                [1, 0, ctheta]])    

        J2_inv[3:6, 3:6] = omega

        L_w_dot = np.matmul(np.matmul(J1_inv, J2_inv), Q_dot)

        for i in range(6):
            Theta_dot[i] = L_w_dot[i] / (M[i] * np.cos(Theta[i]) - N[i] * np.sin(Theta[i]))

        #publish
        servo_velocities = ServoAnglesStamped()
        servo_velocities.header.frame_id = "servo"
        servo_velocities.header.stamp = platform_pose.header.stamp
        for i in range(6):
            servo_velocities.Theta.append(np.rad2deg(Theta_dot[i]))
        self.pub_servo_velocities.publish(servo_velocities)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_velocity_kinematics')
    id = InverseDynamics()
    rospy.spin()