#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from delta_2.msg import ServoAngles6DoFStamped
from tf.transformations import euler_from_quaternion

class InverseKinematics:
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

        #init publisher and subscriber
        self.pub_servo_angles = rospy.Publisher('/' + robot_name + '/servo_setpoint/positions', ServoAngles6DoFStamped, queue_size=1) #servo angle publisher
        self.sub_platform_state = rospy.Subscriber('/' + robot_name + '/platform_setpoint/pose', PoseStamped, self.callback) #target pose subscriber
        
    def callback(self, platform_state): #callback calculates servo angles
        #assign positions to vector X
        X = np.asarray([platform_state.pose.position.x, platform_state.pose.position.y, platform_state.pose.position.z])

        print(platform_state.pose)

        #assign rotations to rotation matrix wRp (from platfom to world coordinates)
        (phi, psi, theta) = euler_from_quaternion([platform_state.pose.orientation.x, platform_state.pose.orientation.y, platform_state.pose.orientation.z, platform_state.pose.orientation.w])
        
        print(np.rad2deg(phi))
        print(np.rad2deg(psi))
        print(np.rad2deg(theta))

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
            self.p_w[i,:] = X + np.matmul(wRp, self.p_p[i,:])
            self.L_w[i,:] = self.p_w[i,:] - self.b_w[i,:]
            rl = np.linalg.norm(self.L_w[i,:])
            L = rl**2 - (self.rs**2 - self.ra**2)

            #convert distances to servo angles
            M = 2 * self.ra * self.p_w[i,2]
            N = 2 * self.ra * (np.cos(self.beta[i]) * (self.p_w[i,0] - self.b_w[i,0]) + np.sin(self.beta[i]) * (self.p_w[i,1] - self.b_w[i,1]))
            disc = L / np.sqrt(M**2 + N**2)

            #check solution exists
            if (disc >= 1.0) or (disc <= -1.0):
                self.Theta[i] = np.nan
            else:
                self.Theta[i] = np.arcsin(disc) - np.arctan(N / M)
                self.Theta[i] = np.rad2deg(self.Theta[i])

        #publish if all servo angles have been solved
        if not np.any(np.isnan(self.Theta)):
            servo_angles = ServoAngles6DoFStamped()
            servo_angles.header.frame_id = "base"
            servo_angles.header.stamp = rospy.Time.now()
            servo_angles.Theta1 = self.Theta[0]
            servo_angles.Theta2 = self.Theta[1]
            servo_angles.Theta3 = self.Theta[2]
            servo_angles.Theta4 = self.Theta[3]
            servo_angles.Theta5 = self.Theta[4]
            servo_angles.Theta6 = self.Theta[5]

            self.pub_servo_angles.publish(servo_angles)
        else:
            rospy.logwarn("MANIPULATOR SETPOINT EXCEEDS WORKSPACE")

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('inverse_kinematics')
    ik = InverseKinematics()
    rospy.spin()