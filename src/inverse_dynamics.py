#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from delta_2.msg import ServoAngles6DoFStamped
from tf.transformations import euler_from_quaternion

class InverseDynamics:
    def __init__(self):
        robot_name = "delta_2"

        #get geometry values form parameter server
        rp = rospy.get_param('/' + robot_name + '/base_radius')
        rb = rospy.get_param('/' + robot_name + '/platform_radius')
        sb = rospy.get_param('/' + robot_name + '/base_joint_spacing')
        sp = rospy.get_param('/' + robot_name + '/platform_joint_spacing')
        self.ra = rospy.get_param('/' + robot_name + '/proximal_link_length')
        self.rs = rospy.get_param('/' + robot_name + '/distal_link_length')

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

        #initialise empty numpy arrays
        self.p_w = np.zeros((6,3))
        self.L_w = np.zeros((6,3))
        self.Theta = np.zeros(6)
        self.n = np.zeros((6,3))
        self.J1_inv = np.zeros((6,6))
        self.J2_inv = np.asarray([[np.eye(3),np.zeros((3,3))],[np.zeros((6,3))]]) 
        self.Theta_dot = np.zeros(6)

        #init publisher and subscriber
        self.pub_servo_velocities = rospy.Publisher('/' + robot_name + '/servo_setpoint/velocities', ServoAngles6DoFStamped, queue_size=1) #servo velocity publisher
        self.sub_platform_vel = rospy.Subscriber('/' + robot_name + '/platform_setpoint/twist', TwistStamped, self.vel_callback) #target twist subscriber
        self.sub_platform_pose = rospy.Subscriber('/' + robot_name + '/platform_setpoint/pose', PoseStamped, self.pose_callback) #target pose subscriber

    def pose_callback(self, platform_pose): 
        #assign positions to vector X
        self.X = np.asarray([platform_pose.pose.position.x, platform_pose.pose.position.y, platform_pose.pose.position.z])

        #assign rotations to rotation matrix wRp (from platfom to world coordinates)
        (phi, psi, theta) = euler_from_quaternion([platform_pose.pose.orientation.x, platform_pose.pose.orientation.y, platform_pose.pose.orientation.z, platform_pose.pose.orientation.w])

        self.cphi = np.cos(phi)
        self.sphi = np.sin(phi)
        self.cpsi = np.cos(psi)
        self.spsi = np.sin(psi)
        self.ctheta = np.cos(theta)
        self.stheta = np.sin(theta)

        Rpsi = np.asarray([[self.cpsi, -self.spsi, 0],
                            [self.spsi, self.cpsi, 0],
                            [0, 0, 1]])
    
        Rtheta = np.asarray([[1, 0, 0],
                            [0, self.ctheta, -self.stheta],
                            [0, self.stheta, self.ctheta]])
      
        Rphi = np.asarray([[self.cphi, 0, self.sphi],
                            [0, 1, 0],
                            [-self.sphi, 0, self.cphi]])

        self.wRp = np.matmul(np.matmul(Rpsi, Rtheta), Rphi)
   
    def vel_callback(self, platform_vel): #callback calculates servo velocities
        #assign velocities to vector q_dot
        q_dot = np.asarray([platform_vel.twist.linear.x, platform_vel.twist.linear.y, platform_vel.twist.linear.z,
                            platform_vel.twist.angular.x, platform_vel.twist.angular.y, platform_vel.twist.angular.z])
        
        omega = np.asarray([[0, self.cpsi, self.spsi * self.stheta],
                [0, self.spsi, -self.cpsi * self.stheta],
                [1, 0, self.ctheta]])
        
        self.J2_inv[[3:5],[3:5]] = omega
        
        for i in range(6):
            #calculate distances from platform and base joints
            self.p_w[i,:] = self.X + np.matmul(self.wRp, self.p_p[i,:])
            self.L_w[i,:] = self.p_w[i,:] - self.b_w[i,:]
            rl = np.linalg.norm(self.L_w[i,:])
            L = rl**2 - (self.rs**2 - self.ra**2)

            #convert distances to servo angles
            M[i] = 2 * self.ra * self.p_w[i,2]
            N[i] = 2 * self.ra * (np.cos(self.beta[i]) * (self.p_w[i,0] - self.b_w[i,0]) + np.sin(self.beta[i]) * (self.p_w[i,1] - self.b_w[i,1]))
            Theta = np.arcsin(L / np.sqrt(M[i]**2 + N**2)) - np.arctan(N[i] / M[i])

            n = self.L_w[i,:] / np.linalg.norm(self.L_w[i,:])
            self.J1_inv[:,i] = np.asarray([n,[np.matmul(self.wRp, np.cross(self.p_p[i,:], n[i,:]))]])

        L_w_dot = np.matmul(np.matmul(self.J1_inv, self.J2_inv), q_dot)

        for i in range(6):
            Theta_dot[i] = L_w_dot[i] / (M[i] * np.cos(Theta[i]) - N[i] * np.sin(Theta[i]))

        #publish
        servo_velocities = ServoAngles6DoFStamped()
        servo_velocities.header.frame_id = "base"
        servo_velocities.header.stamp = rospy.Time.now()
        servo_velocities.Theta1 = np.rad2deg(Theta_dot[0])
        servo_velocities.Theta2 = np.rad2deg(Theta_dot[1])
        servo_velocities.Theta3 = np.rad2deg(Theta_dot[2])
        servo_velocities.Theta4 = np.rad2deg(Theta_dot[3])
        servo_velocities.Theta5 = np.rad2deg(Theta_dot[4])
        servo_velocities.Theta6 = np.rad2deg(Theta_dot[5])
        self.pub_servo_velocities.publish(servo_velocities)

 

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_dynamics')
    id = InverseDynamics()
    rospy.spin()