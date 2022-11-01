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
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Stabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        #init publishers and subscribers
        tooltip_pose_sub = rospy.Subscriber(
            '/tooltip_setpoint/pose', PoseStamped, self.tip_pose_callback, tcp_nodelay=True)
        sub_tooltip_twist = rospy.Subscriber(
            '/tooltip_setpoint/velocity', TwistStamped, self.tip_twist_callback, tcp_nodelay=True)
        manipulator_state_sub = rospy.Subscriber(
            '/manipulator/state', String, self.state_callback, tcp_nodelay=True)     
        drone_pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.drone_pose_callback, tcp_nodelay=True)
        sub_drone_twist = rospy.Subscriber(
            '/mavros/local_position/velocity_body', TwistStamped, self.drone_twist_callback, tcp_nodelay=True)
        
        self.pub_platform_pose = rospy.Publisher(
            '/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_platform_twist = rospy.Publisher(
            '/platform_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)

        # initial values of stuff
        self.stabilising_mode = "RETRACTED"
        self.home_pos, home_rot = self._lookup_vec_and_rot('stewart_base', 'workspace_center', Timeout=10)
        self.DT_d, self.d_R_t = self._lookup_vec_and_rot('base_link', 'tooltip_init', Timeout=10)
        self.PT_p, self.p_R_t = self._lookup_vec_and_rot('platform', 'tooltip', Timeout=10)
        self.retracted_pos = np.asarray(rospy.get_param('/retracted_position'))

        self.V_sp_d = np.asarray([0,0,0])
        self.Omega_sp_d = np.asarray([0,0,0])
               
        rospy.spin()

    def state_callback(self, state):
        self.stabilising_mode = state.data

    def tip_pose_callback(self, tooltip_pose_msg):
        #tooltip setpoint position in world frame
        self.WT_w_offb, self.w_R_t_offb = self._msg_to_vec_and_rot(tooltip_pose_msg)
        if self.stabilising_mode == "STAB_6DOF":
            self._publish_tf_from_pose(tooltip_pose_msg, 'map', 'tooltip_setpoint')
        
    def tip_twist_callback(self, sub_tooltip_twist):
        #setpoint tooltip velocity in world frame
        self.V_sp_d = np.asarray([sub_tooltip_twist.twist.linear.x, sub_tooltip_twist.twist.linear.y, sub_tooltip_twist.twist.linear.z])
        #setpoint tooltip angular velocity in world frame
        self.Omega_sp_d = np.asarray([sub_tooltip_twist.twist.angular.x, sub_tooltip_twist.twist.angular.y, sub_tooltip_twist.twist.angular.z])

    def drone_pose_callback(self, sub_drone_pose):
        WD_w, w_R_d = self._msg_to_vec_and_rot(sub_drone_pose)               
        
        if self.stabilising_mode == "RETRACTED":
            self.BP_b = self.retracted_pos
            b_R_p = R.from_quat([0,0,0,1])     
        elif self.stabilising_mode =="HOME":
            self.BP_b = self.home_pos
            b_R_p = R.from_quat([0,0,0,1])
        elif self.stabilising_mode == "STAB_3DOF": 
            self.DB_d, self.d_R_b = self._lookup_vec_and_rot('base_link', 'stewart_base', Timeout=10)  
            d_R_w = w_R_d.inv()
            b_R_d = self.d_R_t.inv()
            b_R_p = b_R_d * d_R_w * self.d_R_b
            yaw = self._extract_yaw_quat(w_R_d)
            b_R_p_noyaw = b_R_p * yaw
            DB_b = b_R_d.apply(self.DB_d)
            self.BP_b = b_R_p_noyaw.apply(self.home_pos + DB_b) - DB_b
            b_R_p = b_R_p_noyaw
        elif self.stabilising_mode == "STAB_6DOF": 
            BT_b, b_R_t = self._lookup_vec_and_rot('stewart_base', 'tooltip_setpoint')
            self.BP_b = BT_b - b_R_t.apply(self.PT_p)
            b_R_p = b_R_t

        #fill in pose msg and publish
        platform_pose = PoseStamped()
        platform_pose.header.frame_id = 'stewart_base'
        platform_pose.header.stamp = sub_drone_pose.header.stamp
        platform_pose.pose.position.x = self.BP_b[0]
        platform_pose.pose.position.y = self.BP_b[1]
        platform_pose.pose.position.z = self.BP_b[2]
        b_q_p = b_R_p.as_quat()
        platform_pose.pose.orientation.x = b_q_p[0]
        platform_pose.pose.orientation.y = b_q_p[1]
        platform_pose.pose.orientation.z = b_q_p[2]
        platform_pose.pose.orientation.w = b_q_p[3]
        self.pub_platform_pose.publish(platform_pose)

    def drone_twist_callback(self, sub_drone_twist):
        V_drone_d = np.asarray([sub_drone_twist.twist.linear.x, sub_drone_twist.twist.linear.y, sub_drone_twist.twist.linear.z])
        Omega_drone_d = np.asarray([sub_drone_twist.twist.angular.x, sub_drone_twist.twist.angular.y, sub_drone_twist.twist.angular.z])

        b_R_d = self.d_R_t.inv()

        V_drone_b = b_R_d.apply(V_drone_d)
        Omega_drone_b = b_R_d.apply(Omega_drone_d)
        V_sp_b = b_R_d.apply(self.V_sp_d)
        Omega_sp_b = b_R_d.apply(self.Omega_sp_d)

        X_sp_b = self.d_R_t.apply(self.BP_b)

        V_plat_b = V_drone_b + np.cross(Omega_drone_b, X_sp_b)

        if self.stabilising_mode == "RETRACTED":
            X_dot = np.asarray([0.0, 0.0, 0.0])
            Omega = np.asarray([0.0, 0.0, 0.0])
        elif self.stabilising_mode =="HOME":
            X_dot = np.asarray([0.0, 0.0, 0.0])
            Omega = np.asarray([0.0, 0.0, 0.0])
        elif self.stabilising_mode == "STAB_3DOF":
            X_dot = -V_plat_b
            Omega = -Omega_drone_b
        elif self.stabilising_mode == "STAB_6DOF":
            X_dot = -V_plat_b + V_sp_b
            Omega = -Omega_drone_b + Omega_sp_b
         
        platform_twist = TwistStamped()
        platform_twist.header.frame_id = 'stewart_base'
        platform_twist.header.stamp = sub_drone_twist.header.stamp
        platform_twist.twist.linear.x = X_dot[0]
        platform_twist.twist.linear.y = X_dot[1]
        platform_twist.twist.linear.z = X_dot[2]
        platform_twist.twist.angular.x = Omega[0]
        platform_twist.twist.angular.y = Omega[1]
        platform_twist.twist.angular.z = Omega[2]
        self.pub_platform_twist.publish(platform_twist)

    def _publish_tf_from_vec_and_rot(self, vector, rotation, frame_id, child_frame_id):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = vector[0]
        pose.pose.position.y = vector[1]
        pose.pose.position.z = vector[2]
        quat = rotation.as_quat()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self._publish_tf_from_pose(pose, frame_id, child_frame_id)

    def _publish_tf_from_pose(self, pose, frame_id, child_frame_id):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation.x = pose.pose.orientation.x
        t.transform.rotation.y = pose.pose.orientation.y
        t.transform.rotation.z = pose.pose.orientation.z
        t.transform.rotation.w = pose.pose.orientation.w
        self.tfBroadcaster.sendTransform(t)

    def _lookup_vec_and_rot(self, frame_id, child_frame_id, Timeout=0.1):
        tf = self.tfBuffer.lookup_transform(frame_id, child_frame_id, time=rospy.Time.now(), timeout=rospy.Duration(Timeout))       
        translation, rotation = self._msg_to_vec_and_rot(tf)
        return translation, rotation

    def _msg_to_vec_and_rot(self, msg):
        if isinstance(msg, TransformStamped):
            translation = np.asarray([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            quat = np.asarray([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
            rotation = R.from_quat(quat)
        elif isinstance(msg, PoseStamped):
            translation = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            quat = np.asarray([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            rotation = R.from_quat(quat)
        return translation, rotation

    def _extract_yaw_quat(self, drone_rot):
        drone_quat = drone_rot.as_quat()
        drone_angles = euler_from_quaternion(drone_quat)
        yaw_quat = quaternion_from_euler(0, drone_angles[2], 0)
        yaw_rot = R.from_quat(yaw_quat)
        return yaw_rot

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('stabilisation')
    S = Stabilisation()