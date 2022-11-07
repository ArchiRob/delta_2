#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped, TransformStamped, Transform, Twist, WrenchStamped, Vector3, Vector3Stamped, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse
from tricopter.srv import *

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

def handle_fetch_poses(req):
    # service fetches waypoints from the ros parameter server for a given print layer. The format
    # should be as follows:

    # prefix:
    #   n_layers: N
    #   layer0:
    #       point0: [x, y, z, roll, pitch, yaw]
    #       ...     [x, y, z, roll, pitch, yaw]
    #       pointN: [x, y, z, roll, pitch, yaw]
    #   ...
    #   layerN:
    #       point0: [x, y, z, roll, pitch, yaw]
    #       ...

    # rospy.loginfo("Fetching poses from .yaml file")
    param_list = rospy.get_param_names()
    
    poses = PoseArray()
    poses.header.frame_id = req.frame_id
    poses.header.stamp = rospy.Time.now()

    if int(rospy.get_param("/"+str(req.prefix)+"/n_layers")) < req.layer_number:
        rospy.logwarn("Requested layer is greater than layers specified in print")

    points = list(filter(lambda k: str(req.prefix) in k, param_list))
    
    points = list(filter(lambda k: ("/layer" + str(req.layer_number)+"/") in k, points))

    if not points:
        rospy.logwarn("No waypoints found - please check .yaml file") 
    else:
        for i in range(len(points)):
            try:
                point_ref = list(filter(lambda k: ("/point" + str(i)) in k, points))  
                point = rospy.get_param(str(point_ref[0]))
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = point[2] #z axis is reversed to match manipulator sign convention
                q = quaternion_from_euler(point[3], point[4], point[5])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses.poses.append(pose)
            except:
                rospy.logwarn("Error extracting waypoints - please check .yaml file")
                break

    resp = fetchPosesResponse()
    resp.poses = poses
    # rospy.loginfo("Poses returned")
    return resp
    
def handle_TOPPRA_trajectory(req):
    # service generates a smooth, interpolated, time-optimal trajectory from an array of poses using TOPPRA package

    num_poses = len(req.poses.poses)
    
    rospy.loginfo("Trajectory requested. Interpolating " + str(num_poses) + " poses at " + str(req.frequency) + "Hz.")

    way_pts = np.zeros((num_poses, 6))
    for i in range(num_poses):
        (roll, pitch, yaw) = euler_from_quaternion([req.poses.poses[i].orientation.x,
                                                    req.poses.poses[i].orientation.y,
                                                    req.poses.poses[i].orientation.z,
                                                    req.poses.poses[i].orientation.w])
        way_pts[i,:] = [req.poses.poses[i].position.x, req.poses.poses[i].position.y, req.poses.poses[i].position.z, roll, pitch, yaw]
    
    ss = np.linspace(0, 1, num_poses)
    
    amax = req.max_acc
    vmax = req.max_vel
    max_yawrate = np.deg2rad(req.max_yawrate)
    max_yawrate_dot = np.deg2rad(req.max_yawrate_dot)
    vlims = [vmax, vmax, vmax, max_yawrate, max_yawrate, max_yawrate]
    alims = [amax, amax, amax, max_yawrate_dot, max_yawrate_dot, max_yawrate_dot]
    
    path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    traj = instance.compute_trajectory(0, 0)

    n_samples = int(traj.duration * req.frequency)
    ts_sample = np.linspace(0, traj.duration, n_samples)
    qs_sample = traj(ts_sample, 0) #position setpoints
    qds_sample = traj(ts_sample, 1) #velocity setpoints
    qdds_sample = traj(ts_sample, 2) #acceleration setpoints

    resp = TOPPRATrajectoryResponse()
    resp.trajectory = MultiDOFJointTrajectory()
    resp.trajectory.header.frame_id = req.poses.header.frame_id
    resp.trajectory.header.stamp = rospy.Time.now()

    for i in range(n_samples):
        trans = Transform()
        trans.translation.x = qs_sample[i,0]
        trans.translation.y = qs_sample[i,1]
        trans.translation.z = qs_sample[i,2]
        q = quaternion_from_euler(qs_sample[i,3], qs_sample[i,4], qs_sample[i,5])
        trans.rotation.x = q[0]
        trans.rotation.y = q[1]
        trans.rotation.z = q[2]
        trans.rotation.w = q[3]

        vel = Twist()
        vel.linear.x = qds_sample[i,0]
        vel.linear.y = qds_sample[i,1]
        vel.linear.z = qds_sample[i,2]
        vel.angular.x = qds_sample[i,3]
        vel.angular.y = qds_sample[i,4]
        vel.angular.z = qds_sample[i,5]

        accel = Twist()
        accel.linear.x = qdds_sample[i,0]
        accel.linear.y = qdds_sample[i,1]
        accel.linear.z = qdds_sample[i,2]
        accel.angular.x = qdds_sample[i,3]
        accel.angular.y = qdds_sample[i,4]
        accel.angular.z = qdds_sample[i,5]

        trajectory_point = MultiDOFJointTrajectoryPoint()
        trajectory_point.transforms.append(trans)
        trajectory_point.velocities.append(vel)
        trajectory_point.accelerations.append(accel)
        trajectory_point.time_from_start = rospy.Duration(i / req.frequency)

        resp.trajectory.points.append(trajectory_point)

    rospy.loginfo("Trajectory ready")
    return resp

def trajectory_server():
    rospy.init_node('trajectory_services')
    # services for trajectory generation
    TOPPRA_service = rospy.Service(
        'get_TOPPRA_trajectory', TOPPRATrajectory, handle_TOPPRA_trajectory)
    fetch_poses_service = rospy.Service(
        'fetch_poses', fetchPoses, handle_fetch_poses)
    rospy.spin()

if __name__ == "__main__":
    trajectory_server()