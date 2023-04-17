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

class NoGoalFoundException(Exception):
    '''Raised when no goal could be found'''
    pass

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
        """ Clears the currently followed trajectory, and loads the new one from the message
        """
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        
    def odom_callback(self, msg):
        """ Updates the heading of the car based on the provided odometry data
        """
        self.car_pose = msg.pose.pose

    def find_goal(self, pt1, pt2):
        """ calculates goal point given two trajectory points
        """
        Q = self.car_pose
        # Q = np.array([5.5, 4.5])
        r = self.lookahead
        P1 = np.array([pt1.pose.position.x, pt1.pose.position.y])
        V = np.array([pt2.pose.position.x, pt2.pose.position.y]) - P1
        # print('Q: ', Q)
        # print('r: ', r)
        # print('P1: ', P1)
        # print('V: ', V)

        a = V.dot(V)
        b = 2 * V.dot(P1 - Q)
        c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r ** 2
        # print('a: ', a)
        # print('b: ', b)
        # print('c: ', c)

        disc = b ** 2 - 4 * a * c
        if disc < 0:
            # print('No Path Found')
            rospy.loginfo('No Path Found')
            raise NoGoalFoundException

        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        # print(t1, t2)

        goal_1 = P1 + t1 * V
        goal_2 = P1 + t2 * V
        # print(goal_1)
        # print(goal_2)

        goal = self.break_tie(goal_1, goal_2, np.array([pt2.pose.position.x, pt2.pose.position.y]))
        # print(goal)
        return goal

    def break_tie(self, pt1, pt2, next):
        """ break tie when two points lie along the circle
        """
        # dist_1 = np.sqrt((pt1.point.x - next.point.x)**2 + (pt1.point.y - next.point.y)**2)
        # dist_2 = np.sqrt((pt2.point.x - next.point.x)**2 + (pt2.point.y - next.point.y)**2)
        dist_1 = np.sqrt((pt1[0] - next[0])**2 + (pt1[1] - next[1])**2)
        dist_2 = np.sqrt((pt2[0] - next[0])**2 + (pt2[1] - next[1])**2)
        if dist_1 < dist_2:
            return pt1
        return pt2

    def drive_to_goal(self, goal_point):
        """ Navigates the car toward the goal point
        
            Args:
                A numpy array containing two numbers, x and y coords of the goal point
        """
        
        if self.car_pose is None:
            return
        
        # define car position
        car_x = self.car_pose.position.x
        car_y = self.car_pose.position.y
        
        # define goal position
        goal_x = goal_point[0]
        goal_y = goal_point[1]
        
        # normalize coords so as to place car at artificial origin
        car_x -= car_x
        goal_x -= car_x
        
        car_y -= car_y
        goal_y -= car_y
        
        # determine drive angle (taken from parking controller)
        drive_angle = np.arctan2(goal_y, goal_x)
        
        # publish the drive command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = drive_angle
        drive_cmd.drive.speed = self.speed
        self.drive_pub(drive_cmd)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()

    # IGNORE: BRADY TESTING SHIT
    # point1 = PoseStamped()
    # point1.pose.position.x = 0
    # point1.pose.position.y = 0
    # point1.pose.position.z = 0
    # point2 = PoseStamped()
    # point2.pose.position.x = 10
    # point2.pose.position.y = 10
    # point2.pose.position.z = 0
    # pt1v = np.array([point1.pose.position.x, point1.pose.position.y])
    # pt2v = np.array([point2.pose.position.x, point2.pose.position.y])
    # pf.find_goal(point1, point2)

