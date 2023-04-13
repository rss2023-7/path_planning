#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        self.start = None
        self.end = None
        self.map = None
        self.resolution = None


    def map_cb(self, msg):
        new_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        new_resolution = msg.info.resolution
        if self.map != new_map or self.resolution != new_resolution:
            self.map = new_map
            self.resolution = new_resolution
            self.try_plan_path()


    def odom_cb(self, msg):
        new_start = self.get_pose(msg.pose.pose)
        if self.start != new_start:
            self.start = new_start
            self.try_plan_path()


    def goal_cb(self, msg):
        new_end = self.get_pose(msg.pose)
        if self.end != new_end:
            self.end = new_end
            self.try_plan_path()


    def get_pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        theta, _, _ = trans.rotation_from_matrix(trans.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
        return (x, y, theta)


    def try_plan_path():
        if self.start != None and self.end != None and self.map != None:
            self.plan_path(self.start, self.end, self.map)

    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
