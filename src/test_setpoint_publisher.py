#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

from dynamic_reconfigure.server import Server
from delta_2.cfg import TestSetpointConfig

def config_callback(config, level): 
    rospy.loginfo("Calculating new position.....")

    q = quaternion_from_euler(np.deg2rad(config.phi), np.deg2rad(config.psi), np.deg2rad(config.theta))

    setpoint = PoseStamped()
    setpoint.header.frame_id = "base"
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.orientation.x = q[0]
    setpoint.pose.orientation.y = q[1]
    setpoint.pose.orientation.z = q[2]
    setpoint.pose.orientation.w = q[3]
    setpoint.pose.position.x = config.x
    setpoint.pose.position.y = config.y
    setpoint.pose.position.z = config.z

    setpoint_pub.publish(setpoint)
    rospy.loginfo("Updated position!")
    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('test_setpoint_publisher')
    robot_name = "delta_2"
    setpoint_pub = rospy.Publisher('/' + robot_name + '/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
    srv = Server(TestSetpointConfig, config_callback)
    rospy.spin()