#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
import tf.transformations as trans
from scipy import ndimage


class Node:
    def __init__(self, cur, parent=None):
        self.cur = cur
        self.x, self.y = cur
        self.parent = parent


class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        print("init planner")

        self.start = None
        self.end = None
        self.map = None
        self.resolution = None
        self.origin = None
        self.is_planning = False
        # self.rng = np.random.default_rng()

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    def map_cb(self, msg):
        new_map = ndimage.binary_dilation(np.array(msg.data).reshape(msg.info.height, msg.info.width).T, iterations=12).astype(float) * 100
        new_resolution = msg.info.resolution
        new_origin = self.get_origin(msg.info.origin)
        if self.map is None or self.map != new_map or self.res is None or self.resolution != new_resolution or self.origin is None or self.origin != new_origin:
            print("got map")
            self.map = new_map
            self.resolution = new_resolution
            self.origin = new_origin
            self.try_plan_path()


    def odom_cb(self, msg):
        new_start = self.get_pose(msg.pose.pose)
        if self.start != new_start:
            self.start = new_start
            self.try_plan_path()


    def goal_cb(self, msg):
        new_end = self.get_pose(msg.pose)
        if self.end != new_end:
            print("got end")
            self.end = new_end
            self.try_plan_path()


    def get_pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        theta, _, _ = trans.rotation_from_matrix(trans.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
        return (x, y, theta)


    def get_origin(self, pose):
        x = pose.position.x
        y = pose.position.y
        rot = trans.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[:3,:3]
        return (x, y, rot)


    def try_plan_path(self):
        if not self.is_planning and self.start is not None and self.end is not None and self.map is not None:
            self.plan_path(self.start, self.end, self.map)


    def dist(self, node, sample):
        return np.sqrt((node[0]-sample[0])**2 + (node[1]-sample[1])**2)


    def path_clear(self, state, end, map):
        for cell in self.get_cells(state, end):
            if not (map[cell] == 0):
                return False
        return True
    

    def convert_to_cell(self, point):
        res = (np.matmul(np.array([[point[0]], [point[1]], [0]]).T, self.origin[2]) + np.array([[self.origin[0]], [self.origin[1]], [0]]).T)/self.resolution
        return (res[0,0], res[0,1])


    def convert_to_point(self, cell):
        res = np.matmul(np.array([[cell[0]], [cell[1]], [0]]).T * self.resolution, np.linalg.inv(self.origin[2])) + np.array([[self.origin[0]], [self.origin[1]], [0]]).T
        return (res[0,0], res[0,1])
    

    def get_cells(self, start, end):
        u = start
        v = (end[0]-start[0], end[1]-start[1])
        X, Y = (int(u[0]), int(u[1]))
        stepX = 1 if v[0] > 0 else -1 if v[0] < 0 else 0
        stepY = 1 if v[1] > 0 else -1 if v[1] < 0 else 0
        tMaxX = (np.round((u[0]*2+stepX)/2) - u[0]) / v[0]
        tMaxY = (np.round((u[1]*2+stepY)/2) - u[1]) / v[1]
        tDeltaX = abs(1/v[0])
        tDeltaY = abs(1/v[1])
        cells = [(X, Y)]
        while tMaxX < 1 or tMaxY < 1:
            if tMaxX < tMaxY:
                tMaxX += tDeltaX
                X += stepX
            else:
                tMaxY += tDeltaY
                Y += stepY
            cells.append((X, Y))
        return cells


    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##
        print("planning path")
        self.is_planning = True
        self.trajectory.clear()

        start_node = Node(self.convert_to_cell(start_point))
        costs = {start_node:0}
        end_pose = (self.convert_to_cell(end_point))

        i = 0
        done = None
        while True:
            sample = end_pose if done is None and (i==0 or np.random.random_sample() < .05) else (np.random.random_sample() * map.shape[0], np.random.random_sample() * map.shape[1])
            if map[tuple(np.floor(sample).astype(int))] == 0:
                # sampled pose is in a free node
                nearest = min(costs.keys(), key=lambda node: self.dist(node.cur, sample))
                dist = self.dist(nearest.cur, sample)
                max_dist = 10.
                if dist > max_dist:
                    scale = max_dist/dist
                    sample = (nearest.cur[0] + (sample[0]-nearest.cur[0])*scale, nearest.cur[1] + (sample[1]-nearest.cur[1])*scale)
                    dist = self.dist(nearest.cur, sample)
                if map[tuple(np.floor(sample).astype(int))] == 0:
                    if self.path_clear(nearest.cur, sample, map):
                        near_nodes = filter(lambda node: self.dist(node.cur, sample) <= max_dist, costs.keys())
                        min_node = nearest
                        min_cost = costs[nearest] + dist
                        clear_nodes = set()
                        for near_node in near_nodes:
                            new_cost = costs[near_node] + self.dist(near_node.cur, sample)
                            if self.path_clear(near_node.cur, sample, map):
                                clear_nodes.add(near_node)
                                if new_cost < min_cost:
                                    min_node = near_node
                                    min_cost = new_cost
                        sample_node = Node(sample, min_node)
                        costs[sample_node] = min_cost
                        for near_node in clear_nodes:
                            new_cost = costs[sample_node] + self.dist(near_node.cur, sample)
                            if new_cost < costs[near_node]:
                                near_node.parent = sample_node
                                costs[near_node] = new_cost
                        if sample == end_pose:
                            done = i
                            end_node = sample_node
            if done is not None and i > done*1.1:
                cur_node = end_node
                path = []
                while cur_node is not None:
                    path.append(cur_node.cur)
                    cur_node = cur_node.parent
                for node in path[::-1]:
                    self.trajectory.addPoint(Node(self.convert_to_point(node)))
                break
            i += 1
        
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

        self.is_planning = False


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
