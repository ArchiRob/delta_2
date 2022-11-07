#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_handler import *

def trajectory():
    max_vel = 0.05
    rospy.init_node('trajectory', anonymous=True)
    layer = 0
    nlayers = rospy.get_param("/waypoints/n_layers")
    complete = True
    tH = trajectoryHandler(frequency=50, max_vel=max_vel, max_acc=1000, max_yawrate=0.001, max_yawrate_dot=0.001)
    tH.generate_print_layer(layer)

    rospy.loginfo("Starting trajectory in 5 secs...")
    rospy.sleep(5)
    
    pub_pose = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=10)
    pub_vel = rospy.Publisher('/platform_setpoint/velocity', TwistStamped, queue_size=10)

    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if layer < nlayers:
            tooltip_pose, tooltip_twist, tooltip_accel, complete = tH.follow_print_trajectory()
            pub_pose.publish(tooltip_pose)
            pub_vel.publish(tooltip_twist)
            if complete: 
                tH.generate_print_layer(layer)
                rospy.loginfo("Layer "+str(layer)+" complete!")
                layer += 1
                complete = False             
        else:
            rospy.loginfo("DONE!")
        rate.sleep()

if __name__ == '__main__':
    try:
        trajectory()
    except rospy.ROSInterruptException:
        pass