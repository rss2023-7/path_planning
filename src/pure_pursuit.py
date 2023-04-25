#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf
import tf.transformations as trans
import tf2_ros


from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

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
        self.speed            = 2.0

        # init transform listener to get ground truth car location for sim only
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # didn't we measure this for the safety controller?
        self.wheelbase_length = 0.8
        
        self.cur_traj = (0, 1) #indexes of points for current trajectory segment

        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        #self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.car_pose = None

        # for visualization
        self.goal_point_pub = rospy.Publisher("/goal_point", Marker, queue_size=1)

        # for analytics
        self.path_error_pub = rospy.Publisher("/path_error", Float32, queue_size=1)
        self.delta_path_error_pub = rospy.Publisher("/delta_path_error", Float32, queue_size=1)
        self.prev_error = 0
        
        self.trajectory_received = False

    def trajectory_callback(self, msg):
        """ Clears the currently followed trajectory, and loads the new one from the message
        """
        

        # print "Receiving new trajectory:", len(msg.poses), "points"
        if self.trajectory_received:
            return

        self.trajectory_received = True

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        
    def odom_callback(self, msg):
        """ Updates the heading of the car based on the provided odometry data
        """
        if len(self.trajectory.points) > 1:

            self.car_pose = msg.pose.pose

            self.update_traj(self.car_pose)

            # print("car is at position (", self.car_pose.position.x, ", ", self.car_pose.position.y, "), between points ", self.cur_traj)

            while True:
                try:
                    goal_point = self.find_goal(self.trajectory.points[self.cur_traj[0]], self.trajectory.points[self.cur_traj[1]])
                    if goal_point is None:
                        if self.cur_traj[1] == len(self.trajectory.points)-1:
                            goal_point = np.array([self.trajectory.points[self.cur_traj[1]][0], self.trajectory.points[self.cur_traj[1]][1]])
                            # print(goal_point)
                            break
                        else:
                            self.cur_traj = (self.cur_traj[0]+1, self.cur_traj[0]+2)
                    else:
                        break
                except:
                    return
                
            marker = Marker()
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5


            marker.header.frame_id = "map"
            marker.pose.position.x = goal_point[0]
            marker.pose.position.y = goal_point[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1

            self.goal_point_pub.publish(marker)

            self.drive_to_goal(goal_point)
    
    def update_traj(self, car_pose):
        '''Updates self.cur_traj with best trajectory segment
        '''
        path = self.trajectory.points[self.cur_traj[0]:]
        def dist_to_seg2(pt1, pt2, car):
            """ calculates the distance from car's position to a line segment
            """
            # p1x = pt1.pose.position.x
            # p1y = pt1.pose.position.y
            # p2x = pt2.pose.position.x
            # p2y = pt2.pose.position.y
            # cx = car.pose.position.x
            # cy = car.pose.position.y

            p1x = pt1[0]
            p1y = pt1[1]
            p2x = pt2[0]
            p2y = pt2[1]
            cx = car.position.x
            cy = car.position.y

            #https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment/1501725#1501725
            dist2 = lambda x1, x2, y1, y2: ((x2 - x1)**2 + (y2 - y1)**2)
            l2 = dist2(p1x, p2x, p1y, p2y)
            if l2 == 0:
                return dist2(p1x, cx, p1y, cy)
            t = ((cx - p1x) * (p2x - p1x) + (cy - p1y) * (p2y - p1y))/l2
            if t >= 1:
                return np.inf
            t = max(0, min(1, t))
            return dist2(cx, p1x + t * (p2x - p1x), cy, p1y + t * (p2y - p1y))
        
        if len(path) > 1:
            dists = np.array([dist_to_seg2(path[i], path[i+1], car_pose) for i in range(len(path)-1)])
            val = np.argmin(dists)
            self.cur_traj = (val, val+1)

            # publish error here
            ground_truth_pose = self.get_ground_truth_pose()
            dists = np.array([dist_to_seg2(path[i], path[i+1], ground_truth_pose) for i in range(len(path)-1)])
            val = np.argmin(dists)
            self.error_pub.publish(dists[val])
            #self.prev_error = dists[val]

            # self.cur_traj[0] = np.argmin(dists)
            # self.cur_traj[1] = self.cur_traj[0] + 1

    def find_goal(self, pt1, pt2):
        """ calculates goal point given two trajectory points
        """
        Q = np.array([self.car_pose.position.x, self.car_pose.position.y])
        r = self.lookahead
        P1 = np.array([pt1[0], pt1[1]])
        V = np.array([pt2[0], pt2[1]]) - P1

        a = V.dot(V)
        b = 2 * V.dot(P1 - Q)
        c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r ** 2

        disc = b ** 2 - 4 * a * c
        if disc < 0:
            rospy.loginfo('No Path Found')
            raise NoGoalFoundException

        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        if t1 > 1: # line doesn't intersect circle
            return None
        t2 = (-b - sqrt_disc) / (2 * a)
        if t2 > 1: # line doesn't intersect circle
            return None

        goal_1 = P1 + t1 * V
        goal_2 = P1 + t2 * V

        goal = self.break_tie(goal_1, goal_2, np.array([pt2[0], pt2[1]]))
        return goal

    def break_tie(self, pt1, pt2, next_pt):
        """ break tie when two points lie along the circle
        """
        # dist_1 = np.sqrt((pt1.point.x - next.point.x)**2 + (pt1.point.y - next.point.y)**2)
        # dist_2 = np.sqrt((pt2.point.x - next.point.x)**2 + (pt2.point.y - next.point.y)**2)
        dist_1 = np.sqrt((pt1[0] - next_pt[0])**2 + (pt1[1] - next_pt[1])**2)
        dist_2 = np.sqrt((pt2[0] - next_pt[0])**2 + (pt2[1] - next_pt[1])**2)
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
        
        drive_cmd = AckermannDriveStamped()
        
        # stop the car if
        # a) current goal point is the final point in the trajectory
        # and
        # b) car is within some acceptable distance of the current goal point
        if (self.car_pose.position.x - self.trajectory.points[-1][0]) ** 2 + (self.car_pose.position.y - self.trajectory.points[-1][1]) ** 2 <= 0.5:
            drive_cmd.drive.steering_angle = 0
            drive_cmd.drive.speed = 0
            self.trajectory_received = False

        # otherwise, navigate to the current goal point
        else:

            # transform from map to car
            rot = trans.quaternion_matrix([self.car_pose.orientation.x, self.car_pose.orientation.y, self.car_pose.orientation.z, self.car_pose.orientation.w])[:3,:3]
            goal_coords = np.matmul(np.array([[goal_point[0]], [goal_point[1]], [0]]).T - np.array([[self.car_pose.position.x], [self.car_pose.position.y], [0]]).T, rot)

            # determine drive angle (taken from parking controller)
            drive_angle = np.arctan2(goal_coords[0, 1], goal_coords[0, 0])
            drive_cmd.drive.steering_angle = drive_angle
            drive_cmd.drive.speed = self.speed
        
        # publish the drive command
        # print("publishing drive cmd with angle = "+str(drive_angle)+" and speed = "+str(self.speed))
        self.drive_pub.publish(drive_cmd)

    def get_ground_truth_pose(self):
        '''
        Returns the ground truth pose of the car.

        Only to be used when running the car in simulation.
        '''
        ground_truth_pose = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time())

        pose = PoseStamped()
        pose.pose.position.x = ground_truth_pose.transform.translation.x
        pose.pose.position.y = ground_truth_pose.transform.translation.y
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        return pose.pose

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

