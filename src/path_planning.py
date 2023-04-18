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
import heapdict




class Node:
    def __init__(self, cur, g_cost, h_cost, parent=None):
        self.cur = cur
        self.x, self.y = cur
        self.parent = parent
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost


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

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    def map_cb(self, msg):
        new_map = ndimage.binary_dilation(np.array(msg.data).reshape(msg.info.height, msg.info.width).T, iterations=3).astype(float) * 100
        new_resolution = msg.info.resolution
        new_origin = self.get_origin(msg.info.origin)
        if self.map is None or self.map != new_map or self.res is None or self.resolution != new_resolution or self.origin is None or self.origin != new_origin:
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


    def get_neighbors(self, node, map):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_x, new_y = int(node.x + dx), int(node.y + dy)
                if 0 <= new_x < map.shape[0] and 0 <= new_y < map.shape[1] and map[new_x, new_y] == 0:
                    neighbor = Node((new_x, new_y), node)
                    neighbors.append(neighbor)
        return neighbors



    def h_cost(self, node, goal):
        return abs(node.x - goal.x) + abs(node.y - goal.y)


    def plan_path(self, start_point, end_point, map):
        self.trajectory.clear()

        start_node = Node(self.convert_to_cell(start_point))
        end_node = Node(self.convert_to_cell(end_point))

        open_list = heapdict()
        open_list[start_node] = 0

        closed_list = set()

        g_cost = {start_node: 0}
        f_cost = {start_node: self.dist(start_node.cur, end_node.cur)}

        while open_list:
            current_node, _ = open_list.popitem()

            if current_node == end_node:
                path = []
                while current_node is not None:
                    path.append(current_node)
                    current_node = current_node.parent
                for node in path[::-1]:
                    self.trajectory.addPoint(Node(self.convert_to_point(node.cur)))
                break

            closed_list.add(current_node)

            neighbors = self.get_neighbors(current_node, map)
            for neighbor in neighbors:
                if neighbor in closed_list:
                    continue

                tentative_g_cost = g_cost[current_node] + self.dist(current_node.cur, neighbor.cur)

                if neighbor not in open_list or tentative_g_cost < g_cost[neighbor]:
                    neighbor.parent = current_node
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + self.dist(neighbor.cur, end_node.cur)
                    open_list[neighbor] = f_cost[neighbor]

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()




if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
