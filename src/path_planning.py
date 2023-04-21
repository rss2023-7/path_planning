#!/usr/bin/env python

import rospy
import math
import numpy as np
from heapq import heappush, heappop
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseArray
import tf.transformations as trans
from utils import LineTrajectory

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

        #publish a point that visualizes in RVIZ the map origin
        origin = PoseStamped()
        origin.header.frame_id = "map"
        origin.pose.position.x = map.info.origin.position.x
        origin.pose.position.y = map.info.origin.position.y
        origin.pose.position.z = map.info.origin.position.z
        origin.pose.orientation.x = map.info.origin.orientation.x
        origin.pose.orientation.y = map.info.origin.orientation.y
        origin.pose.orientation.z = map.info.origin.orientation.z
        origin.pose.orientation.w = map.info.origin.orientation.w
        self.origin_pub.publish(origin)
        rospy.loginfo("Map origin: {}".format(map.info))

     

        

        # rospy.loginfo("Map received: {}".format(self.map_received))
        # rospy.loginfo("Map size: {} x {}".format(map.info.width, map.info.height))
        # rospy.loginfo("Map resolution: {}".format(map.info.resolution))
        # rospy.loginfo("Map origin: {}".format(map.info.origin))
        # rospy.loginfo("Map array: {}".format(self.map_array))
        # rospy.loginfo("Map meta: {}".format(self.map_meta))
        # #log the resolution data and origin data from map_meta
        # rospy.loginfo("Map resolution from meta: {}".format(self.map_meta.resolution))


    def goal_cb(self, end):
        self.end = self.get_pose(end.pose)
        rospy.loginfo("Goal received: {}".format(self.end))
        if self.map_received and hasattr(self, 'start'):
            self.plan_path(self.start, self.end, self.loaded_map)

    def odom_cb(self, odom):
        self.start = self.get_pose(odom.pose.pose)
        # rospy.loginfo("Start received: {}".format(self.start))
        if self.map_received and hasattr(self, 'end'):
            self.plan_path(self.start, self.end, self.loaded_map)



    def world_to_grid(self, x, y, map):
        u = int((x - map.info.origin.position.x) / map.info.resolution)
        v = int((y - map.info.origin.position.y) / map.info.resolution)
        # rospy.loginfo("Got to world_to_grid func, Map origin: {}".format(map.info.origin))

        return u, v

    def plan_path(self, start_point, end_point, map):
        # rospy.loginfo("Got to plan_path func, Map origin: {}".format(map.info.origin))
        # rospy.loginfo("Start point: {}".format(start_point))
        # rospy.loginfo("End point: {}".format(end_point))

         
        start_u, start_v = self.world_to_grid(start_point[0], start_point[1], map)
        end_u, end_v = self.world_to_grid(end_point[0], end_point[1], map)
        
    
        # rospy.loginfo("Start u: {}".format(start_u))
        # rospy.loginfo("Start v: {}".format(start_v))
        # rospy.loginfo("End u: {}".format(end_u))
        # rospy.loginfo("End v: {}".format(end_v))


        def euclidean_distance(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        def reconstruct_path(came_from, current):
            path = deque([current])
            while current in came_from:
                current = came_from[current]
                path.appendleft(current)
            return list(path)

        def get_neighbors(u, v):
            #log the dimensions of the map array
            # rospy.loginfo("Map array dimensions: {}".format(self.map_array.shape))
            # #log dimensions of map.info width and height
            # rospy.loginfo("Map info width: {}".format(map.info.width))
            # rospy.loginfo("Map info height: {}".format(map.info.height))
            # #log the value of u and v
            # rospy.loginfo("U: {}".format(u))
            # rospy.loginfo("V: {}".format(v))
            # #log the value of map_array[v, u]
            # rospy.loginfo("Map array occupancy: {}".format(self.map_array[0, 0]))
            # # log the min and max indexes of the map array
            # rospy.loginfo("Map array min index: {}".format(self.map_array.min()))
            # rospy.loginfo("Map array max index: {}".format(self.map_array.max()))
            neighbors = []
            for du, dv in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                if 0 <= u + du < map.info.width and 0 <= v + dv < map.info.height:
                    rospy.loginfo("Map array occupancy: {}".format(self.map_array[v + dv, u + du]))
                    if self.map_array[v + dv, u + du] < 50:
                        neighbors.append((u + du, v + dv))
            # rospy.loginfo("Neighbors: {}".format(neighbors))
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
        else:
            # rospy.loginfo("No path found!")
            return

        # rospy.loginfo("Path found!")
        self.trajectory.clear()
        for u, v in path:
            x = u * map.info.resolution + map.info.origin.position.x
            y = v * map.info.resolution + map.info.origin.position.y
            self.trajectory.addPoint(Point(x, y, 0))

        traj_msg = self.trajectory.toPoseArray()
        self.traj_pub.publish(traj_msg)

        self.trajectory.publish_viz()

if __name__ == "__main__":
    rospy.init_node("path_planner")
    path_planner = PathPlan()
    rospy.spin()
