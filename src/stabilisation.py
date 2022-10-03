#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Vector3, AccelStamped
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion, quaternion_from_euler
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
        self.pub_platform_accel = rospy.Publisher('/platform_setpoint/accel', AccelStamped, queue_size=1, tcp_nodelay=True)
        
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
                self.manip_mode = "HOME"
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
        #initialise position vectors and rotation quaternions
        #tooltip setpoint position in world frame
        OT_w = self.X_sp_w
        #rotation of tooltip/platform in world frame
        w_q_p = self.Q_sp_w
         #measured drone base_link position in world frame
        OD_w = np.asarray([sub_drone_pose.pose.position.x, sub_drone_pose.pose.position.y, sub_drone_pose.pose.position.z])
        
        PT_p = np.asarray([self.p_tf_t.transform.translation.x, self.p_tf_t.transform.translation.y, self.p_tf_t.transform.translation.z]) 

        DB_d = np.asarray([self.d_tf_b.transform.translation.x, self.d_tf_b.transform.translation.y, self.d_tf_b.transform.translation.z])

        #measured drone base_link orientation in world frame
        w_q_d = np.asarray([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
        d_q_b = np.asarray([self.d_tf_b.transform.rotation.x, self.d_tf_b.transform.rotation.y, self.d_tf_b.transform.rotation.z, self.d_tf_b.transform.rotation.w])
        b_q_d = quaternion_conjugate(d_q_b)
        p_q_w = quaternion_conjugate(w_q_p)
        
        d_q_w = quaternion_conjugate(w_q_d)
        # can definitely make this line more efficient but it works :p
        b_q_p = quaternion_multiply(quaternion_multiply(w_q_p, quaternion_multiply(b_q_d, d_q_w)), d_q_b)
        
        BT_d = -DB_d + quaternion_rotation(OT_w - OD_w, d_q_w)
        BP_b = quaternion_rotation(BT_d, b_q_d) - quaternion_rotation(PT_p, b_q_p)

        drone_angles = euler_from_quaternion([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
        yaw = quaternion_from_euler(0,drone_angles[2], 0)
    
        DB_b = quaternion_rotation(DB_d, b_q_d)
        if self.manip_mode == "RETRACTED":
            self.X = self.retracted_pos
            self.Q = np.asarray([0, 0, 0, 1])        
        elif self.manip_mode =="HOME":
            self.X = self.home_pos
            self.Q = np.asarray([0, 0, 0, 1])
        elif self.manip_mode == "STAB_3DOF":
            self.X = quaternion_rotation(self.home_pos + DB_b, quaternion_multiply(b_q_p, yaw)) - DB_b
            self.Q = quaternion_multiply(b_q_p, yaw)
        elif self.manip_mode == "STAB_6DOF":
            self.X = BP_b
            self.Q = b_q_p
    
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
        V_drone_d = Vector3Stamped(sub_drone_twist.header, Vector3(sub_drone_twist.twist.linear.x, sub_drone_twist.twist.linear.y, sub_drone_twist.twist.linear.z))
        Omega_drone_d = Vector3Stamped(sub_drone_twist.header, Vector3(sub_drone_twist.twist.angular.x, sub_drone_twist.twist.angular.y, sub_drone_twist.twist.angular.z))
        
        d_tf_b = self.tfBuffer.lookup_transform('stewart_base', 'base_link', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))
        V_drone_b = do_transform_vector3(V_drone_d, d_tf_b)
        Omega_drone_b = do_transform_vector3(Omega_drone_d, d_tf_b)

        d_tf_t = self.tfBuffer.lookup_transform('base_link', 'tooltip', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))

        DT_d = Vector3Stamped(sub_drone_twist.header, Vector3(d_tf_t.transform.translation.x, d_tf_t.transform.translation.y, d_tf_t.transform.translation.z))
        DT_b = do_transform_vector3(DT_d, d_tf_b)

        V_drone_b = np.asarray([V_drone_b.vector.x, V_drone_b.vector.y, V_drone_b.vector.z])
        Omega_drone_b = np.asarray([Omega_drone_b.vector.x, Omega_drone_b.vector.y, Omega_drone_b.vector.z])
        DT_b = np.asarray([DT_b.vector.x, DT_b.vector.y, DT_b.vector.z])

        V_sp_d = Vector3Stamped(sub_drone_twist.header, Vector3(self.V_sp[0], self.V_sp[1], self.V_sp[2])) 
        V_sp_b = do_transform_vector3(V_sp_d, d_tf_b)
        V_sp_b = np.asarray([V_sp_b.vector.x, V_sp_b.vector.y, V_sp_b.vector.z])

        Omega_sp_d = Vector3Stamped(sub_drone_twist.header, Vector3(self.Omega_sp[0], self.Omega_sp[1], self.Omega_sp[2])) 
        Omega_sp_b = do_transform_vector3(Omega_sp_d, d_tf_b)
        Omega_sp_b = np.asarray([Omega_sp_b.vector.x, Omega_sp_b.vector.y, Omega_sp_b.vector.z])

        V_tip_b = V_drone_b + np.cross(Omega_drone_b, DT_b)

        Omega_tip_b = Omega_drone_b

        if self.manip_mode == "RETRACTED":
            X_dot = np.asarray([0.0, 0.0, 0.0])
            Omega = np.asarray([0.0, 0.0, 0.0])
        elif self.manip_mode =="HOME":
            X_dot = np.asarray([0.0, 0.0, 0.0])
            Omega = np.asarray([0.0, 0.0, 0.0])
        elif self.manip_mode == "STAB_3DOF":
            X_dot = -V_tip_b
            Omega = -Omega_tip_b
        elif self.manip_mode == "STAB_6DOF":
            X_dot = -V_tip_b + V_sp_b
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


    def drone_accel_callback(self, sub_drone_accel):
        A_d = Vector3Stamped(sub_drone_accel.header, sub_drone_accel.linear_acceleration)
        A_b = do_transform_vector3(A_d, self.d_tf_b)

        if self.manip_mode == "RETRACTED":
            X_ddot = Vector3(0.0, 0.0, 0.0)
        elif self.manip_mode == "HOME":
            X_ddor = Vector3(0.0, 0.0, 0.0)
        elif self.manip_mode == "STAB_3DOF":
            X_ddot = A_b.vector
        elif self.manip_mode == "STAB_6DOF":
            X_ddot = A_b.vector

        platform_accel = AccelStamped()
        platform_accel.header.frame_id = 'stewart_base'
        platform_accel.header.stamp = sub_drone_accel.header.stamp
        platform_accel.accel.linear = X_ddot
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