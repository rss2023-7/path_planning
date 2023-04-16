#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
import tf.transformations as trans


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

        self.start = None
        self.end = None
        self.map = None
        self.resolution = None
        self.origin = None
        # self.rng = np.random.default_rng()

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    def map_cb(self, msg):
        # print(type(msg.data))
        # print(len(msg.data))
        new_map = np.array(msg.data).reshape(msg.info.height, msg.info.width).T
        new_resolution = msg.info.resolution
        new_origin = self.get_origin(msg.info.origin)
        if self.map != new_map or self.resolution != new_resolution or self.origin != new_origin:
            # print(new_map.shape)
            # print(msg.info.width, msg.info.height)
            # print(new_origin)
            # print(new_resolution)
            # print("got map")
            self.map = new_map
            self.resolution = new_resolution
            self.origin = new_origin
            self.try_plan_path()


    def odom_cb(self, msg):
        new_start = self.get_pose(msg.pose.pose)
        if self.start != new_start:
            # print("got start")
            self.start = new_start
            self.try_plan_path()


    def goal_cb(self, msg):
        new_end = self.get_pose(msg.pose)
        if self.end != new_end:
            # print("got end")
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
        if self.start is not None and self.end is not None and self.map is not None:
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
        self.trajectory.clear()

        # print(start_point)
        # print(end_point)
        # print(self.origin)
        # print(self.resolution)
        start_node = Node(self.convert_to_cell(start_point))
        costs = {start_node:0}
        end_pose = (self.convert_to_cell(end_point))
        # print(start_node.cur)
        # print(end_pose)

        # print(tuple(np.floor(start_node.cur).astype(int)))
        # print(tuple(np.floor(end_pose).astype(int)))
        # print(map[tuple(np.floor(start_node.cur).astype(int))])
        # print(map[tuple(np.floor(end_pose).astype(int))])

        first_sample = True
        i = 0
        while True:
            sample = end_pose if first_sample or np.random.random_sample() < .05 else (np.random.random_sample() * map.shape[0], np.random.random_sample() * map.shape[1])
            # print(sample)
            # print(sample==end_pose)
            # if sample == end_pose:
            #     print(i)
            # if (i % 1000)==0:
            #     print(costs)
            if map[tuple(np.floor(sample).astype(int))] == 0:
                # sampled pose is in a free node
                nearest = min(costs.keys(), key=lambda node: self.dist(node.cur, sample))
                dist = self.dist(nearest.cur, sample)
                max_dist = 5.
                # if (i%100) == 0:
                #     print("before:",sample)
                #     print(i)
                #     print(sample == end_pose)
                #     print("near:",nearest.cur)
                if dist > max_dist:
                    scale = max_dist/dist
                    sample = (nearest.cur[0] + (sample[0]-nearest.cur[0])*scale, nearest.cur[1] + (sample[1]-nearest.cur[1])*scale)
                    dist = self.dist(nearest.cur, sample)
                # print(dist)
                # if (i%100) == 0:
                #     print("after:",sample)
                # self.trajectory.addPoint(Node(self.convert_to_point(start_node.cur)))
                # self.trajectory.addPoint(Node(self.convert_to_point(sample)))
                # break
                if map[tuple(np.floor(sample).astype(int))] == 0:
                    # print("close good")
                    # print(nearest.cur, sample)
                    if self.path_clear(nearest.cur, sample, map):
                        # print("path good")
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
                            cur_node = sample_node
                            path = []
                            while cur_node is not None:
                                path.append(cur_node.cur)
                                cur_node = cur_node.parent
                            for node in path[::-1]:
                                self.trajectory.addPoint(Node(self.convert_to_point(node)))
                            break
                #     else:
                #         print("path bad")
                # else:
                #     print("close bad")
            # if i % 1000 == 0:
            #     print(len(costs))
            # if len(costs) > 10000:
            #     print("quitting")
            #     for node in costs:
            #         self.trajectory.addPoint(Node(self.convert_to_point(node.cur)))
            #     print("quitting")
            #     break
            first_sample = False
            i += 1
        
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
