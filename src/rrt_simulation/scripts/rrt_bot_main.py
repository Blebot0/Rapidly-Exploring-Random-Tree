#!/usr/bin/env python

import rospy, tf
import numpy as np
import time
from geometry_msgs.msg import *
from std_msgs.msg import *
from gazebo_msgs.srv import SpawnModel
import random
from rrt_simulation.msg import Waypoint
from sensor_msgs.msg import NavSatFix
import math

flag = 0

class RRT_Planner():

    def __init__(self, start_point = None, end_point = None, obstacle_loc = None, goal_prob = None, min_dis = None, max_dis = None, length = None, goal_radius = None):
        
        self.start_point = Node(start_point[0], start_point[1])
        self.end_point = Node(end_point[0], end_point[1])
        self.goal_area = (end_point[0], end_point[1], goal_radius)
        self.obstacle_loc = obstacle_loc
            
        self.goal_prob = goal_prob
        self.final_path = []
        self.min_dis = min_dis
        self.max_dis = max_dis
        
        self.length = length
        self.node_list = [self.start_point]
        
        self.final_path = []
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.waypoint_pub = rospy.Publisher("/waypoints", Waypoint, queue_size=5)
        self.waypoint_bool = rospy.Publisher("/waypoint_bool", Bool, queue_size=1)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.waypoint_obj = Waypoint()
        self.final_path_list = []
        self.num = 1
        self.num_path = 1
    
    def node_spawner(self, locx, locy):
        with open("/home/blebot/mowito_ws/src/rrt_simulation/urdf/marker.urdf", "r") as f:
            product_xml = f.read()
        item_name = "marker_{0}".format(self.num)
        print("Spawning model: {}".format(item_name))
        item_pose   =  Pose(Point(x=locx, y=locy, z=0), Quaternion(0, 0, 0, 0))
        self.spawn_model(item_name, product_xml, "/marker_{}".format(self.num), item_pose, "world")
        self.num += 1

    def path_spawner(self, locx, locy):
        with open("/home/blebot/mowito_ws/src/rrt_simulation/urdf/marker_black.urdf", "r") as f:
            product_xml = f.read()
        item_name = "marker_path_{0}".format(self.num_path)
        print("Spawning model: {}".format(item_name))
        item_pose   =  Pose(Point(x=locx, y=locy, z=0), Quaternion(0, 0, 0, 0))
        self.spawn_model(item_name, product_xml, "/marker_path_{}".format(self.num_path), item_pose, "world")
        self.num_path += 1
    
    def rtt_planner(self):
        global flag
        if flag == 0:
            
            random_node = self.random_node_generator()
            if self.obstacle_collision_detector(random_node):
                return
            
            nearest_node_index = self.nearest_node_finder(self.node_list, random_node)
            nearest_node = Node(self.node_list[nearest_node_index].x, self.node_list[nearest_node_index].y)
            self.chain(nearest_node=nearest_node, random_node=random_node)
            self.node_list.append(random_node)            
            if self.goal_reach(random_node):
                print("Goal Reached")
                flag = 1
                self.final_path = self.generate_goal_path()
                self.chain(self.start_point, self.final_path[-1])
                self.final_path = self.final_course()
                
                for i in self.final_path:
                    self.path_spawner(i.x, i.y)
                    self.waypoint_obj.x_waypoint = round(i.x, 5)
                    self.waypoint_obj.y_waypoint = round(i.y, 5)
                    self.waypoint_pub.publish(self.waypoint_obj)
                self.waypoint_bool.publish(True)
                return

    def final_course(self):
        path = self.final_path
        for node in self.final_path:

            if node.dist > self.length:
                path1 = self.final_path[:self.final_path.index(node)]
                path2 = self.final_path[self.final_path.index(node):]
            
                new_node = Node(node.x, node.y)
                new_node.dist = node.dist
                new_node.theta = node.theta
                new_node.parent_node = node.parent_node
                num_of_waypoints = int(math.floor(node.dist / self.length))
                
                for i in range(0, num_of_waypoints):
                    new_node.x -=  self.length* math.cos(node.theta)
                    new_node.y -= self.length * math.sin(node.theta)
                    temp_node = Node(new_node.x, new_node.y)
                    if self.obstacle_collision_detector(new_node):
                        temp_node.y -= self.length * math.sin(node.theta) * 2
                        temp_node.x += self.length * math.cos(node.theta) * 2
                    
                    new_node.path.append(temp_node)
                path1.extend(new_node.path)
                path1.extend(path2)

                path1.sort(key= lambda node: (node.x - self.start_point.x)** 2 + (node.y - self.start_point.y)**2)
                path = path1

        path = path[:path.index(self.end_point) + 1]
        return path
            
    def obstacle_collision_detector(self, node):
        for center_x, center_y, radius in self.obstacle_loc:
            if (node.x - center_x) ** 2 + (node.y - center_y) ** 2 < radius ** 2:
                return True
        
        else:
            return False

    def goal_reach(self, node):
        center_x, center_y, radius = self.goal_area

        if (node.x - center_x) ** 2 + (node.y - center_y) ** 2 < radius ** 2:
            return True

    
    def generate_goal_path(self):
        path = [self.end_point]
        node = self.node_list[-1]
        while node.parent_node is not None:
            path.append(node)
            node = node.parent_node
        path.append(node)
        return path

    def chain(self, nearest_node, random_node):
        random_node.parent_node = nearest_node
        random_node.dist, random_node.theta = self.calculate_parameter(random_node, nearest_node)


   
    def random_node_generator(self):
        random_num = random.randrange(0, 100)
        x_node = 0.
        y_node = 0.
        if random_num>=0 and random_num < 35:
        
            x_node = random.uniform(self.min_dis * 1 / 3, self.max_dis * 1 / 3)
            y_node = random.uniform(self.min_dis * 1 / 3, self.max_dis * 1 / 3)
    
        elif random_num>=35 and random_num < 70:
                    
            x_node = random.uniform(self.min_dis * 2 / 3, self.max_dis * 2 / 3)
            y_node = random.uniform(self.min_dis * 2 / 3, self.max_dis * 2 / 3)
    
        elif random_num>=70 and random_num <= 100:
                    
            x_node = random.uniform(self.min_dis * 1 , self.max_dis * 1 )
            y_node = random.uniform(self.min_dis * 1 , self.max_dis * 1 )
        
        self.node_spawner(locx=x_node, locy= y_node)
        return Node(x_node, y_node)
    
    
    @staticmethod
    def calculate_parameter(node_initial, node_final):
        dx = node_initial.x - node_final.x
        dy = node_initial.y - node_final.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return dist, theta    
    
    
    @staticmethod
    def nearest_node_finder(node_list, random_node):
        if len(node_list) != 0:
            all_dist = [ (i.x - random_node.x)**2 + (i.y - random_node.y)**2 for i in node_list]
            min_dist_index = all_dist.index(min(all_dist))
            return min_dist_index
    

    @staticmethod
    def long_to_y(input_longitude):
        return -105292.0089353767 * (input_longitude - 8.90003)
    
    @staticmethod
    def lat_to_x(input_latitude):
        return 110692.0702932625 * (input_latitude - 49.90002809)


class Node:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.theta = 0
        self.dist = 0
        self.path = []
        self.parent_node = None




if __name__ == "__main__":
    count = 0
    rospy.init_node("rrt_node_python")

    obstacle_x = RRT_Planner.lat_to_x(49.900073224)
    obstacle_y = RRT_Planner.long_to_y(8.8999880161)
    # Obstacle location (x, y, radius)
    obstacle_loc = [(obstacle_x, obstacle_y, 1)]
    max_iter = 500
    obj = RRT_Planner(goal_prob= 0, min_dis= -10, max_dis=10, obstacle_loc=obstacle_loc, start_point=[0, 0], end_point= [8, 8], length= 2., goal_radius=2)
    
    while not rospy.is_shutdown() and count < max_iter:
        obj.rtt_planner()
        count += 1