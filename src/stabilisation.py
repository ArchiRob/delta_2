#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Vector3
from tf.transformations import quaternion_multiply, quaternion_conjugate
from tf2_geometry_msgs import do_transform_vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu

#code to generate end-effector setpoints accounting for random drone perturbations
class Stabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #obtain transforms from platform to tooltip and drone base_link to platform_base
        
        self.d_tf_b = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.p_tf_t = self.tfBuffer.lookup_transform('platform', 'tooltip', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.home_tf = self.tfBuffer.lookup_transform('stewart_base', 'workspace_center', time=rospy.Time(0), timeout=rospy.Duration(10))

        #init publishers and subscribers
        self.pub_platform_pose = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_platform_twist = rospy.Publisher('/platform_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.pub_platform_accel = rospy.Publisher('/platform_setpoint/accel', Vector3Stamped, queue_size=1, tcp_nodelay=True)
        
        sub_drone_state = rospy.Subscriber('/mavros/state', State, self.state_callback, tcp_nodelay=True) #target pose subscriber
        sub_drone_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback, tcp_nodelay=True)
        sub_drone_twist = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.drone_twist_callback, tcp_nodelay=True)
        sub_drone_accel = rospy.Subscriber('/mavros/imu/data', Imu, self.drone_accel_callback, tcp_nodelay=True)
        sub_tooltip_pose = rospy.Subscriber('/tooltip_setpoint/pose', PoseStamped, self.tip_pose_callback, tcp_nodelay=True)
        sub_tooltip_twist = rospy.Subscriber('/tooltip_setpoint/velocity', TwistStamped, self.tip_twist_callback, tcp_nodelay=True)

        self.manip_mode = "RETRACTED"
        self.home_pos = np.asarray([self.home_tf.transform.translation.x, self.home_tf.transform.translation.y, self.home_tf.transform.translation.z])
        self.retracted_pos = rospy.get_param('/retracted_position')
        self.V_sp = np.asarray([0.0, 0.0, 0.0])
        self.Omega_sp = np.asarray([0.0, 0.0, 0.0])
        self.X_sp_w = np.asarray([0.0, 0.0, 0.0])
        self.Q_sp_W = np.asarray([self.d_tf_b.transform.rotation.x, self.d_tf_b.transform.rotation.y, self.d_tf_b.transform.rotation.z, self.d_tf_b.transform.rotation.w])

    def state_callback(self, state):
        if state.armed:
            if state.mode == "OFFBOARD":
                self.manip_mode = "STAB_6DOF"
            elif state.mode == "POSCTL":
                self.manip_mode = "STAB_3DOF"
            else:
                self.manip_mode = "RETRACTED"
        else:
            self.manip_mode = "RETRACTED"

        print(self.manip_mode)

    def tip_pose_callback(self, sub_tooltip_pose):
        #tooltip setpoint position in world frame
        self.X_sp_w = np.asarray([sub_tooltip_pose.pose.position.x, sub_tooltip_pose.pose.position.y, sub_tooltip_pose.pose.position.z]) 
        #rotation of tooltip/platform in world frame
        self.Q_sp_w = np.asarray([sub_tooltip_pose.pose.orientation.x, sub_tooltip_pose.pose.orientation.y, sub_tooltip_pose.pose.orientation.z, sub_tooltip_pose.pose.orientation.w]) 

    def tip_twist_callback(self, sub_tooltip_twist):
        #setpoint tooltip velocity in world frame
        self.V_sp = np.asarray([sub_tooltip_twist.twist.linear.x, sub_tooltip_twist.twist.linear.y, sub_tooltip_twist.twist.linear.z])
        #setpoint tooltip angular velocity in world frame
        self.Omega_sp = np.asarray([sub_tooltip_twist.twist.angular.x, sub_tooltip_twist.twist.angular.y, sub_tooltip_twist.twist.angular.z])

    def drone_pose_callback(self, sub_drone_pose):
        w_tf_b = self.tfBuffer.lookup_transform('map', 'stewart_base', time=sub_drone_pose.header.stamp, timeout=rospy.Duration(0.01))
        w_q_b = np.asarray([w_tf_b.transform.rotation.x, w_tf_b.transform.rotation.y, w_tf_b.transform.rotation.z, w_tf_b.transform.rotation.w])

        # WB_w = Vector3Stamped(sub_drone_pose.header, Vector3(w_tf_b.transform.translation.x, w_tf_b.transform.translation.y, w_tf_b.transform.translation.z))
        WB_w = np.asarray([w_tf_b.transform.translation.x, w_tf_b.transform.translation.y, w_tf_b.transform.translation.z])

        # platform to tooltip in platform frame
        PT_p = np.asarray([self.p_tf_t.transform.translation.x, self.p_tf_t.transform.translation.y, self.p_tf_t.transform.translation.z])

        # tooltip rotation in base frame
        Q_b = quaternion_multiply(self.Q_sp_w , w_q_b)

        # tooltip vector in base frame
        PT_b = quaternion_rotation(PT_p, Q_b)

        #target vector in base frame
        WT_b = quaternion_rotation(self.X_sp_w, w_q_b)

        #base vector in base frame
        WB_b = quaternion_rotation(WB_w, w_q_b)

        BP_b = -WB_b + WT_b - PT_b

        if self.manip_mode == "RETRACTED":
            self.X = self.retracted_pos
            self.Q = np.asarray([0, 0, 0, 1])
        elif self.manip_mode == "STAB_3DOF":
            self.X = self.home_pos
            self.Q = Q_b
        elif self.manip_mode == "STAB_6DOF":
            self.X = BP_b
            self.Q = Q_b
    
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

    def drone_twist_callback(self, sub_drone_twist):
        V_drone_d = np.asarray([sub_drone_twist.twist.linear.x, sub_drone_twist.twist.linear.y, sub_drone_twist.twist.linear.z])
        Omega_drone_d = np.asarray([sub_drone_twist.twist.angular.x, sub_drone_twist.twist.angular.y, sub_drone_twist.twist.angular.z])

        d_tf_t = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))
        d_tf_b = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))

        DT_d = np.asarray([d_tf_t.transform.translation.x, d_tf_t.transform.translation.y, d_tf_t.transform.translation.z])

        V_tip_d = -(V_drone_d + np.cross(Omega_drone_d, DT_d)) + self.V_sp

        Omega_tip_d = -Omega_drone_d + self.Omega_sp

        V_tip_d = Vector3Stamped(sub_drone_twist.header, Vector3(V_tip_d[0], V_tip_d[1], V_tip_d[2]))
        Omega_tip_d = Vector3Stamped(sub_drone_twist.header, Vector3(Omega_tip_d[0], Omega_tip_d[1], Omega_tip_d[2]))

        V_tip_p = do_transform_vector3(V_tip_d, d_tf_b)
        Omega_tip_p = do_transform_vector3(Omega_tip_d, d_tf_b)

        if self.manip_mode == "RETRACTED":
            X_dot = Vector3(0.0, 0.0, 0.0)
            Omega = Vector3(0.0, 0.0, 0.0)
        elif self.manip_mode == "STAB_3DOF":
            X_dot = Vector3(0.0, 0.0, 0.0)
            Omega = Omega_tip_p.vector
        elif self.manip_mode == "STAB_6DOF":
            X_dot = V_tip_p.vector
            Omega = Omega_tip_p.vector
         
        platform_twist = TwistStamped()
        platform_twist.header.frame_id = 'stewart_base'
        platform_twist.header.stamp = sub_drone_twist.header.stamp
        platform_twist.twist.linear = X_dot
        platform_twist.twist.angular = Omega
        self.pub_platform_twist.publish(platform_twist)


    def drone_accel_callback(self, sub_drone_accel):
        A_d = Vector3Stamped(sub_drone_accel.header, sub_drone_accel.linear_acceleration)
        A_b = do_transform_vector3(A_d, self.d_tf_b)

        if self.manip_mode == "RETRACTED":
            X_ddot = Vector3(0.0, 0.0, 0.0)
        elif self.manip_mode == "STAB_3DOF":
            X_ddot = Vector3(0.0, 0.0, 0.0)
        elif self.manip_mode == "STAB_6DOF":
            X_ddot = A_b

        platform_accel = Vector3Stamped()
        platform_accel.header.frame_id = 'stewart_base'
        platform_accel.header.stamp = sub_drone_accel.header.stamp
        platform_accel.vector = X_ddot
        self.pub_platform_accel.publish(platform_accel)
        

def quaternion_rotation(vector, quaternion):
    #assumes quaternion is already unit magnitude (not a problem when dealing with orientation messages)
    vector = np.append(vector, [0.0])
    quaternion_inverse = quaternion_conjugate(quaternion)
    rotated_vector = quaternion_multiply(quaternion, quaternion_multiply(vector, quaternion_inverse))
    return rotated_vector[0:3]

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('stabilisation')
    S = Stabilisation()
    rospy.spin()