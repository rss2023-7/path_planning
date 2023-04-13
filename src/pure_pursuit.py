#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        
        # these numbers can be played with
        self.lookahead        = 1.5
        self.speed            = 1.0
        
        # didn't we measure this for the safety controller?
        self.wheelbase_length = 0.8
        
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.car_pose = None

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        
    def odom_callback(self, msg):
        ''' Updates the heading of the car based on the provided odometry data
        '''
        self.car_pose = msg.pose.pose
    
    def drive_to_goal(self, goal_pose):
        ''' Navigates the car toward the goal point
        '''
        
        if self.car_pose is None:
            return
        
        # define car position
        car_x = self.car_pose.point.x
        car_y = self.car_pose.point.y
        
        # define goal position
        goal_x = goal_pose.point.x
        goal_y = goal_pose.point.y
        
        # publish the drive command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = 0.0
        drive_cmd.drive.speed = self.speed
        self.drive_pub(drivee_cmd)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
