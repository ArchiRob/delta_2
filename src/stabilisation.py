#!/usr/bin/env python
from re import sub
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

        #obtain transforms and convert to numpy arrays for later
        d_tf_b = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.DB_d = np.asarray([d_tf_b.transform.translation.x, d_tf_b.transform.translation.y, d_tf_b.transform.translation.z])
        self.d_q_b = np.asarray([d_tf_b.transform.rotation.x, d_tf_b.transform.rotation.y, d_tf_b.transform.rotation.z, d_tf_b.transform.rotation.w])

        p_tf_t = self.tfBuffer.lookup_transform('platform', 'tooltip', time=rospy.Time(0), timeout=rospy.Duration(10))
        self.PT_p = np.asarray([p_tf_t.transform.translation.x, p_tf_t.transform.translation.y, p_tf_t.transform.translation.z])
        self.p_q_t = np.asarray([p_tf_t.transform.rotation.x, p_tf_t.transform.rotation.y, p_tf_t.transform.rotation.z, p_tf_t.transform.rotation.w])

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

        # initial values of stuff
        self.manip_mode = "RETRACTED"
        self.home_pos = np.asarray([self.home_tf.transform.translation.x, self.home_tf.transform.translation.y, self.home_tf.transform.translation.z])
        self.retracted_pos = rospy.get_param('/retracted_position')
        self.V_sp_d = np.asarray([0.0, 0.0, 0.0])
        self.Omega_sp_d = np.asarray([0.0, 0.0, 0.0])
        self.X_sp_w = np.asarray([0.0, 0.0, 0.0])
        self.Q_sp_W = self.d_q_b
        self.X = self.retracted_pos

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

    def tip_pose_callback(self, sub_tooltip_pose):
        #tooltip setpoint position in world frame
        self.X_sp_w = np.asarray([sub_tooltip_pose.pose.position.x, sub_tooltip_pose.pose.position.y, sub_tooltip_pose.pose.position.z]) 
        #rotation of tooltip/platform in world frame
        self.Q_sp_w = np.asarray([sub_tooltip_pose.pose.orientation.x, sub_tooltip_pose.pose.orientation.y, sub_tooltip_pose.pose.orientation.z, sub_tooltip_pose.pose.orientation.w]) 

    def tip_twist_callback(self, sub_tooltip_twist):
        #setpoint tooltip velocity in world frame
        self.V_sp_d = np.asarray([sub_tooltip_twist.twist.linear.x, sub_tooltip_twist.twist.linear.y, sub_tooltip_twist.twist.linear.z])
        #setpoint tooltip angular velocity in world frame
        self.Omega_sp_d = np.asarray([sub_tooltip_twist.twist.angular.x, sub_tooltip_twist.twist.angular.y, sub_tooltip_twist.twist.angular.z])

    def drone_pose_callback(self, sub_drone_pose):
        WD_w = np.asarray([sub_drone_pose.pose.position.x, sub_drone_pose.pose.position.y, sub_drone_pose.pose.position.z])
        w_q_d = np.asarray([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
              
        d_q_w = quaternion_conjugate(w_q_d) #rotation from drone frame to world frame      
        b_q_d = quaternion_conjugate(self.d_q_b) #rotation from base frame to drone frame        
        DB_w = quaternion_rotation(self.DB_d, w_q_d) #drone to base vector in world frame        

        if self.manip_mode == "RETRACTED":
            self.X = self.retracted_pos
            self.Q = np.asarray([0, 0, 0, 1])        
        elif self.manip_mode =="HOME":
            self.X = self.home_pos
            self.Q = np.asarray([0, 0, 0, 1])
        elif self.manip_mode == "STAB_3DOF":
            PT_w = quaternion_rotation(self.PT_p, self.d_q_b) #platform to tooltip vector in world frame        
            b_q_w = quaternion_multiply(b_q_d, d_q_w) #base to world rotation
            b_q_p = quaternion_multiply(b_q_w, self.d_q_b)
            drone_angles = euler_from_quaternion([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
            yaw = quaternion_from_euler(0,drone_angles[2], 0)
            DB_b = quaternion_rotation(self.DB_d, b_q_d)
            BP_b = quaternion_rotation(self.home_pos + DB_b, quaternion_multiply(b_q_p, yaw)) - DB_b
            self.X = BP_b 
            self.Q = quaternion_multiply(b_q_p, yaw)
        elif self.manip_mode == "STAB_6DOF":   
            PT_w = quaternion_rotation(quaternion_rotation(self.PT_p, self.d_q_b), self.Q_sp_w) #platform to tooltip vector in world frame        
            b_q_w = quaternion_multiply(b_q_d, d_q_w) #base to world rotation
            b_q_p = quaternion_multiply(b_q_w, quaternion_multiply(self.Q_sp_w, self.d_q_b)) 
            BP_w = -DB_w - WD_w + self.X_sp_w - PT_w #base to platform vector in world frame
          
            BP_b = quaternion_rotation(BP_w, b_q_w) #base to platform vector in base frame       
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
        V_drone_d = np.asarray([sub_drone_twist.twist.linear.x, sub_drone_twist.twist.linear.y, sub_drone_twist.twist.linear.z])
        Omega_drone_d = np.asarray([sub_drone_twist.twist.angular.x, sub_drone_twist.twist.angular.y, sub_drone_twist.twist.angular.z])

        b_q_d = quaternion_conjugate(self.d_q_b)

        V_drone_b = quaternion_rotation(V_drone_d, b_q_d)
        Omega_drone_b = quaternion_rotation(Omega_drone_d, b_q_d)
        V_sp_b = quaternion_rotation(self.V_sp_d, b_q_d)
        Omega_sp_b = quaternion_rotation(self.Omega_sp_d, b_q_d)

        X_sp_b = quaternion_rotation(self.X, self.d_q_b)

        V_plat_b = V_drone_b + np.cross(Omega_drone_b, X_sp_b)

        if self.manip_mode == "RETRACTED":
            X_dot = np.asarray([0.0, 0.0, 0.0])
            Omega = np.asarray([0.0, 0.0, 0.0])
        elif self.manip_mode =="HOME":
            X_dot = np.asarray([0.0, 0.0, 0.0])
            Omega = np.asarray([0.0, 0.0, 0.0])
        elif self.manip_mode == "STAB_3DOF":
            X_dot = -V_plat_b
            Omega = -Omega_drone_b
        elif self.manip_mode == "STAB_6DOF":
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


    def drone_accel_callback(self, sub_drone_accel):
        gravity = quaternion_rotation(np.asarray([0,0,9.805]), quaternion_conjugate(np.asarray([sub_drone_accel.orientation.x, sub_drone_accel.orientation.y, sub_drone_accel.orientation.z, sub_drone_accel.orientation.w])))
        A_d = np.asarray([sub_drone_accel.linear_acceleration.x, sub_drone_accel.linear_acceleration.y, sub_drone_accel.linear_acceleration.z]) - gravity

        b_q_d = quaternion_conjugate(self.d_q_b) #rotation from base frame to drone frame
        A_b = quaternion_rotation(A_d, b_q_d)
        
        if self.manip_mode == "RETRACTED":
            X_ddot = np.asarray([0.0, 0.0, 0.0])
        elif self.manip_mode == "HOME":
            X_ddot = np.asarray([0.0, 0.0, 0.0])
        elif self.manip_mode == "STAB_3DOF":
            X_ddot = -A_b
        elif self.manip_mode == "STAB_6DOF":
            X_ddot = -A_b

        platform_accel = AccelStamped()
        platform_accel.header.frame_id = 'stewart_base'
        platform_accel.header.stamp = sub_drone_accel.header.stamp
        platform_accel.accel.linear.x = X_ddot[0]
        platform_accel.accel.linear.y = X_ddot[1]
        platform_accel.accel.linear.z = X_ddot[2]
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