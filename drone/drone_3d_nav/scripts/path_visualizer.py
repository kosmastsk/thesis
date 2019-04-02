#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import sys
import time


def truthCallback(data):
    global xAnt
    global yAnt

    pose = PoseStamped()

    pose.header.frame_id = "map"
    pose.pose.position.x = float(data.pose.pose.position.x)
    pose.pose.position.y = float(data.pose.pose.position.y)
    pose.pose.position.z = float(data.pose.pose.position.z)
    pose.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
        pose.header.seq = path_truth.header.seq + 1
        path_truth.header.frame_id = "map"
        path_truth.header.stamp = rospy.Time.now()
        pose.header.stamp = path_truth.header.stamp
        path_truth.poses.append(pose)
        # Published the msg

    pub_truth.publish(path_truth)

    xAnt = pose.pose.orientation.x
    yAnt = pose.pose.position.y
    return path_truth


def amclCallback(data):
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
        pose.header.seq = path_amcl.header.seq + 1
        path_amcl.header.frame_id = "map"
        path_amcl.header.stamp = rospy.Time.now()
        pose.header.stamp = path_amcl.header.stamp
        path_amcl.poses.append(pose)
        # Published the msg

    pub_amcl.publish(path_amcl)

    xAnt = pose.pose.orientation.x
    yAnt = pose.pose.position.y
    return path_amcl


if __name__ == '__main__':
    # Initializing global variables
    global xAnt
    global yAnt
    global cont
    xAnt = 0.0
    yAnt = 0.0

    # Initializing node
    rospy.init_node('path_visualizer')

    pub_amcl = rospy.Publisher('/path/amcl_pose', Path, queue_size=1)
    pub_truth = rospy.Publisher('/path/truth', Path, queue_size=1)

    path_amcl = Path()
    path_truth = Path()
    msg_amcl = PoseStamped()
    msg_truth = PoseStamped()

    # Subscription to the required odom topic (edit accordingly)
    msg_amcl = rospy.Subscriber('/amcl_pose', PoseStamped, amclCallback)
    msg_truth = rospy.Subscriber('/ground_truth/state', Odometry,
                                 truthCallback)

    rate = rospy.Rate(30)  # 30hz

    try:
        while not rospy.is_shutdown():
            # rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
