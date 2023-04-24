#!/usr/bin/env python

import rospy
import math
import numpy as np
from heapq import heappush, heappop
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path, Odometry, GridCells
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from std_msgs.msg import Header, ColorRGBA
import tf.transformations as trans
from visualization_msgs.msg import Marker
from utils import LineTrajectory
from trajectory_builder import BuildTrajectory

class PathPlan:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.map_received = False
        self.map_array = None
        self.map_meta = None
        self.loaded_map = None
        self.origin_pub = rospy.Publisher("/map_origin", PoseStamped, queue_size=10)
        self.goal_pub = rospy.Publisher("/goal_point", PoseStamped, queue_size=10)
        self.start_point_pub = rospy.Publisher("/start_point", PoseStamped, queue_size=10)
        self.start_marker_pub = rospy.Publisher("/path_markers/start", Marker, queue_size=20)
        self.end_marker_pub = rospy.Publisher("/path_markers/end", Marker, queue_size=20)
        self.neighbor_grid_pub = rospy.Publisher("/neighbor_grid", GridCells, queue_size=10)
        self.visited_nodes_pub = rospy.Publisher("/visited_grids", GridCells, queue_size=20)

        self.counter = 0

    def get_pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        theta, _, _ = trans.rotation_from_matrix(trans.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
        return (x, y, theta)
    
    def map_cb(self, map):
        self.map_array = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))
        self.map_meta = map.info
        self.map_received = True
        self.loaded_map = map






    def goal_cb(self, end):
        self.end = self.get_pose(end.pose)
        self.counter += 1
        if self.map_received and hasattr(self, 'start'):
            self.plan_path(self.start, self.end, self.loaded_map)

    def odom_cb(self, odom):
        self.start = self.get_pose(odom.pose.pose)
        self.counter += 1
        if self.map_received and hasattr(self, 'end'):
            self.plan_path(self.start, self.end, self.loaded_map)
 

    def world_to_grid(self, x, y, map):
        max_world_x_coord = map.info.origin.position.x
        min_world_y_coord = -(map.info.height * map.info.resolution - map.info.origin.position.y)
        u = int(abs(y - min_world_y_coord) / map.info.resolution)
        v = int(abs(x - max_world_x_coord) / map.info.resolution)
        return u, v
    
    def grid_to_world(self, u, v, map):
        max_world_x_coord = map.info.origin.position.x
        min_world_y_coord = -(map.info.height * map.info.resolution - map.info.origin.position.y)
        x = max_world_x_coord - (v * map.info.resolution)
        y = (u * map.info.resolution) + (min_world_y_coord) 
        return x, y
    
    

    def plan_path(self, start_point, end_point, map):
        start_u, start_v = self.world_to_grid(start_point[0], start_point[1], map)
        end_u, end_v = self.world_to_grid(end_point[0], end_point[1], map)

        def visualize_marker(u, v, map, publisher):
            point = self.grid_to_world(u, v, map)
            marker = Marker()
            header = Header()
            header.frame_id = "map"
            marker.header = header
            marker.id = 0
            marker.type = 2
            marker.action = 0
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            publisher.publish(marker)
        
        visualize_marker(start_u, start_v, map, self.start_marker_pub)
        visualize_marker(end_u, end_v, map, self.end_marker_pub)
        
        def create_visited_node_gridcells(u, v, map):
            point = self.grid_to_world(u, v, map)
            grid_cells = GridCells()
            grid_cells.header.frame_id = "map"
            grid_cells.cell_width = map.info.resolution
            grid_cells.cell_height = map.info.resolution
            grid_cells.cells.append(Point(point[0], point[1], 0))
            return grid_cells





        def euclidean_distance(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        def reconstruct_path(came_from, current):
            path = deque([current])
            while current in came_from:
                current = came_from[current]
                path.appendleft(current)
            return list(path)

        def get_neighbors(u, v):
            neighbors = []
            for du, dv in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                if 0 <= u + du < map.info.width and 0 <= v + dv < map.info.height:
                    if self.map_array[v + dv, u + du] < 50:
                        neighbors.append((u + du, v + dv))
            return neighbors

        frontier = []
        heappush(frontier, (0, (start_u, start_v)))
        came_from = {}
        cost_so_far = {(start_u, start_v): 0}

        while frontier:
            _, current = heappop(frontier)
            if current == (end_u, end_v):
                path = reconstruct_path(came_from, current)
                break
            for neighbor in get_neighbors(*current):
                new_cost = cost_so_far[current] + euclidean_distance(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + euclidean_distance(neighbor, (end_u, end_v))
                    heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current
                    visited_node_marker = create_visited_node_gridcells(neighbor[0], neighbor[1], map)
                    self.visited_nodes_pub.publish(visited_node_marker)

        else:
            # rospy.loginfo("No path found!")
            return

        # rospy.loginfo("Path found!")
        self.trajectory.clear()
        for u, v in path:
            x, y = self.grid_to_world(u, v, map)

            self.trajectory.addPoint(Point(x, y, 0))

        traj_msg = self.trajectory.toPoseArray()
        self.traj_pub.publish(traj_msg)

        self.trajectory.publish_viz()

if __name__ == "__main__":
    rospy.init_node("path_planner")
    path_planner = PathPlan()
    rospy.spin()
