#!/usr/bin/env python3

#------------------------------------------------------------------------------------------------
# STABILISATION
# I make no apologies for using the english spelling of 'stabilise'
# This code works out the manipulator setpoints to stabilise against random motion about a
# pre-defined centre (using the tf package). I had all sorts of issues with making closed 
# loops when trying to use the tf package to do this so I threw my hands up and worked out
# all the coordinate rotations with pen and paper and lots of trial and error. You have 
# absolutely no chance of following the maths here and I'm not 100% sure I understand it
# myself.
#------------------------------------------------------------------------------------------------

import rospy
import numpy as np
import tf2_ros
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Stabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #obtain transforms and convert to numpy arrays for later
        d_tf_b = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.DB_d = [d_tf_b.transform.translation.x, d_tf_b.transform.translation.y, d_tf_b.transform.translation.z]
        self.d_R_b = R.from_quat([d_tf_b.transform.rotation.x, d_tf_b.transform.rotation.y, d_tf_b.transform.rotation.z, d_tf_b.transform.rotation.w])

        p_tf_t = self.tfBuffer.lookup_transform('platform', 'tooltip', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.PT_p = [p_tf_t.transform.translation.x, p_tf_t.transform.translation.y, p_tf_t.transform.translation.z]
        self.p_R_t = R.from_quat([p_tf_t.transform.rotation.x, p_tf_t.transform.rotation.y, p_tf_t.transform.rotation.z, p_tf_t.transform.rotation.w])

        d_tf_ti = self.tfBuffer.lookup_transform('base_link', 'tooltip_init', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.DTi_d = [d_tf_ti.transform.translation.x, d_tf_ti.transform.translation.y, d_tf_ti.transform.translation.z]
        self.d_R_ti = R.from_quat([d_tf_ti.transform.rotation.x, d_tf_ti.transform.rotation.y, d_tf_ti.transform.rotation.z, d_tf_ti.transform.rotation.w])

        self.home_tf = self.tfBuffer.lookup_transform('stewart_base', 'workspace_center', time=rospy.Time(0), timeout=rospy.Duration(10))

        #init publishers and subscribers
        sub_manipulator_state = rospy.Subscriber('/manipulator/state', String, self.state_callback, tcp_nodelay=True)    
        self.pub_platform_pose = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        sub_tooltip_pose = rospy.Subscriber('/tooltip_setpoint/pose', PoseStamped, self.tip_pose_callback, tcp_nodelay=True)
        sub_drone_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback, tcp_nodelay=True)
        
        sp_raw_sub = rospy.Subscriber(
            '/mavros/setpoint_raw/target_local', PositionTarget, self.sp_raw_cb, queue_size=1, tcp_nodelay=True)

        # initial values of stuff
        self.manip_mode = "RETRACTED"
        self.home_pos = [self.home_tf.transform.translation.x, self.home_tf.transform.translation.y, self.home_tf.transform.translation.z]
        self.retracted_pos = rospy.get_param('/retracted_position')
        self.WT_w = [0.0, 0.0, 0.0]
        self.w_R_p = self.d_R_b
        self.BT_b = self.retracted_pos
        self.tip_R = self.w_R_p
        self.tip_pos = [0.0, 0.0, 0.0]
        self.WD_w = [0.0, 0.0, 0.0]
        self.drone_raw_pos = [0.0, 0.0, 0.0]
        rospy.spin()

    def state_callback(self, state):
        self.manip_mode = state.data

    def tip_pose_callback(self, sub_tooltip_pose):
        #tooltip setpoint position in world frame
        self.WT_w = [sub_tooltip_pose.pose.position.x, sub_tooltip_pose.pose.position.y, sub_tooltip_pose.pose.position.z]
        #rotation of tooltip/platform in world frame
        # print([sub_tooltip_pose.pose.orientation.x, sub_tooltip_pose.pose.orientation.y, sub_tooltip_pose.pose.orientation.z, sub_tooltip_pose.pose.orientation.w])
        self.w_R_p = R.from_quat([sub_tooltip_pose.pose.orientation.x, sub_tooltip_pose.pose.orientation.y, sub_tooltip_pose.pose.orientation.z, sub_tooltip_pose.pose.orientation.w]) 

    def sp_raw_cb(self, sp_raw):
        drone_raw_pos = [sp_raw.position.x, sp_raw.position.y, sp_raw.position.z]
        self.drone_raw_vel = [sp_raw.velocity.x, sp_raw.velocity.y, sp_raw.velocity.z]
        for i in range(3):
            if np.isnan(drone_raw_pos[i]):
                self.drone_raw_pos[i] = self.WD_w[i]
            elif self.drone_raw_vel[i] < 0.2:
                self.drone_raw_pos[i] += self.drone_raw_vel[i] * (rospy.Time.to_sec(rospy.Time.now()) - rospy.Time.to_sec(sp_raw.header.stamp))
            else:
                self.drone_raw_pos[i] = drone_raw_pos[i]

        drone_yaw_quat = quaternion_from_euler(0, 0, sp_raw.yaw)
        self.drone_raw_R = R.from_quat(drone_yaw_quat) 

    def drone_pose_callback(self, sub_drone_pose):
        self.WD_w = np.asarray([sub_drone_pose.pose.position.x, sub_drone_pose.pose.position.y, sub_drone_pose.pose.position.z])
        w_R_d = R.from_quat([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
              
        d_R_w = w_R_d.inv() #rotation from drone frame to world frame      
        b_R_d = self.d_R_b.inv() #rotation from base frame to drone frame        
        DB_w = w_R_d.apply(self.DB_d) #drone to base vector in world frame   

        if self.manip_mode == "RETRACTED":
            self.X = self.retracted_pos
            self.Q = [0, 0, 0, 1]      
        elif self.manip_mode =="HOME":
            self.X = self.home_pos
            self.Q = [0, 0, 0, 1]
        elif self.manip_mode == "STAB_3DOF":
            PT_w = self.d_R_b.apply(self.PT_p) #platform to tooltip vector in world frame        
            b_R_w = b_R_d * d_R_w #base to world rotation
            b_R_p = b_R_w * self.d_R_b
            drone_angles = euler_from_quaternion([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
            yaw = quaternion_from_euler(0,drone_angles[2], 0)
            yaw = R.from_quat(yaw)
            DB_b = b_R_d.apply(self.DB_d)
            BP_b = (b_R_p * yaw).apply(self.home_pos + DB_b) - DB_b
            self.X = BP_b 
            self.Q = (b_R_p * yaw).as_quat()
        elif self.manip_mode == "STAB_6DOF":   
            PT_w = self.w_R_p.apply(self.d_R_b.apply(self.PT_p)) #platform to tooltip vector in world frame     
            b_R_w = b_R_d * d_R_w #base to world rotation
            b_R_p = b_R_w * self.w_R_p
            BP_w = -DB_w - self.WD_w + self.WT_w - PT_w #base to platform vector in world frame
            BP_b = b_R_w.apply(BP_w) #base to platform vector in base frame       
            self.X = BP_b
            self.Q = b_R_p.as_quat()
        elif self.manip_mode == "STAB_6DOF-MANUAL":  
            PT_w = self.drone_raw_R.apply(self.d_R_b.apply(self.PT_p)) #platform to tooltip vector in world frame
            WT_w = self.drone_raw_pos + self.drone_raw_R.apply(self.DTi_d) #tooltip position in world frame
            b_R_w = b_R_d * d_R_w #base to world rotation
            b_R_p = b_R_w * self.drone_raw_R
            BP_w = -DB_w - self.WD_w + WT_w - PT_w #base to platform vector in world frame
            BP_b = b_R_w.apply(BP_w) #base to platform vector in base frame       
            self.X = BP_b
            self.Q = (b_R_p * self.d_R_b).as_quat()

        #fill in pose msg and publish
        platform_pose = PoseStamped()
        platform_pose.header.frame_id = 'stewart_base'
        platform_pose.header.stamp = sub_drone_pose.header.stamp
        platform_pose.pose.position.x = self.X[0]
        platform_pose.pose.position.y = self.X[1]
        platform_pose.pose.position.z = self.X[2]
        platform_pose.pose.orientation.x = self.Q[0]
        platform_pose.pose.orientation.y = self.Q[1]
        platform_pose.pose.orientation.z = self.Q[2]
        platform_pose.pose.orientation.w = self.Q[3]
        self.pub_platform_pose.publish(platform_pose)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('stabilisation')
    S = Stabilisation()