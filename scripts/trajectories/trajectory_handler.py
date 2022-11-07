#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist, PoseArray, PoseStamped, TwistStamped, Transform, Vector3
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tricopter.srv import *
from viz_functions import *

class trajectoryHandler:
    def __init__(self, frequency, max_vel, max_acc, max_yawrate, max_yawrate_dot):
        self.frequency = frequency
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_yawrate = max_yawrate
        self.max_yawrate_dot = max_yawrate_dot
        self.waypoint_prefix = "waypoints"
        self.print_frame_id = "stewart_base"

        self._point_count = 0    
        self._tooltip_trajectory = MultiDOFJointTrajectory()

        self._n_layers = rospy.get_param(str(self.waypoint_prefix)+'/n_layers')

        # publishers for visualisation only
        self._pub_toolpath_viz = rospy.Publisher('/viz/toolpath', Path, queue_size=1)
        self._pub_print_viz = rospy.Publisher('/viz/print', Path, queue_size=1) 

        # publish print visualisation periodically
        rospy.Timer(rospy.Duration(5.0), self._viz_timer_cb, reset=True)

    def follow_print_trajectory(self):
        header = self._tooltip_trajectory.header
        
        tooltip_pose = PoseStamped(header=header)
        tooltip_velocity = TwistStamped(header=header)
        tooltip_acceleration = TwistStamped(header=header)

        if self._point_count < len(self._tooltip_trajectory.points):   
            tooltip_pose, tooltip_velocity, tooltip_acceleration = self._read_trajectory(self._tooltip_trajectory, self._point_count)
            self._point_count += 1
            complete = False
        else: 
            complete = True
            self._point_count = 0
            
        return tooltip_pose, tooltip_velocity, tooltip_acceleration, complete

    def generate_print_layer(self, layer_number):
        self._point_count = 0 #ensure point count is reset in case last trajectory was interrupted
        print_waypoints = self._fetch_waypoints_from_yaml(layer_number)
        self._tooltip_trajectory = self._TOPPRA_interpolation(print_waypoints)
        publish_viz_trajectory(self._tooltip_trajectory, self._pub_toolpath_viz)
        # return drone_trajectory, tooltip_trajectory

    def _read_trajectory(self, trajectory, point_num):
        pose = PoseStamped()
        velocity = TwistStamped()
        acceleration = TwistStamped() 
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = trajectory.header.frame_id
        velocity.header = pose.header
        acceleration.header = pose.header
        pose.pose.position = trajectory.points[point_num].transforms[0].translation
        pose.pose.orientation = trajectory.points[point_num].transforms[0].rotation
        velocity.twist = trajectory.points[point_num].velocities[0]
        acceleration.twist = trajectory.points[point_num].accelerations[0]
        return pose, velocity, acceleration

    def _fetch_waypoints_from_yaml(self, layer_number):
        # get poses from file
        rospy.wait_for_service('fetch_poses')
        get_poses = rospy.ServiceProxy('fetch_poses', fetchPoses)
        request = fetchPosesRequest()
        request.prefix = self.waypoint_prefix
        request.frame_id = self.print_frame_id
        request.layer_number = layer_number
        response = get_poses(request)
        return response.poses

    def _TOPPRA_interpolation(self, poses):
        #interpolate with TOPPRA
        rospy.wait_for_service('get_TOPPRA_trajectory')
        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest()
        request.frequency = self.frequency
        request.max_vel = self.max_vel
        request.max_acc = self.max_acc
        request.max_yawrate = self.max_yawrate
        request.max_yawrate_dot = self.max_yawrate_dot
        request.poses = poses
        response = get_traj(request)
        return response.trajectory

    def _viz_timer_cb(self, event):
        publish_viz_print(self._pub_print_viz)