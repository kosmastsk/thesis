#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import sys

import time


def callback(data):
    global xAnt
    global yAnt

    pose = PoseStamped()

    pose.header.frame_id = "map"
    pose.pose.position.x = float(data.pose.position.x)
    pose.pose.position.y = float(data.pose.position.y)
    pose.pose.position.z = float(data.pose.position.z)
    pose.pose.orientation.x = float(data.pose.orientation.x)
    pose.pose.orientation.y = float(data.pose.orientation.y)
    pose.pose.orientation.z = float(data.pose.orientation.z)
    pose.pose.orientation.w = float(data.pose.orientation.w)

    if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        # Published the msg

    pub.publish(path)

    xAnt = pose.pose.orientation.x
    yAnt = pose.pose.position.y
    return path


if __name__ == '__main__':
    # Initializing global variables
    global xAnt
    global yAnt
    global cont
    xAnt = 0.0
    yAnt = 0.0

    # Initializing node
    rospy.init_node('path_plotter_amclpose')

    pub = rospy.Publisher('/path/amcl_pose', Path, queue_size=1)

    path = Path()
    msg = PoseStamped()

    # Subscription to the required odom topic (edit accordingly)
    msg = rospy.Subscriber('/amcl_pose', PoseStamped, callback)

    rate = rospy.Rate(30)  # 30hz

    try:
        while not rospy.is_shutdown():
            # rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
