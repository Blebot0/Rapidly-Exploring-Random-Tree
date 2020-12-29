#!/usr/bin/env python

import rospy, tf
import numpy as np
import time
from geometry_msgs.msg import *
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
import random
from sensor_msgs.msg import NavSatFix
import math
flag = 0


## Removed Node Spawner from the random generator and put it in the chain only  

class RRT_Planner():

    class Node:
        
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.theta = 0
            self.dist = 0
            self.path = []
            self.parent_node = None


    def __init__(self, start_point = None, end_point = None, obstacle_loc = None, goal_prob = None, min_dis = None, max_dis = None, length = None, waypoint = None, goal_radius = None):
        
        self.start_point = self.Node(start_point[0], start_point[1])
        self.end_point = self.Node(end_point[0], end_point[1])
        self.goal_area = (end_point[0], end_point[1], goal_radius)
        self.obstacle_loc = obstacle_loc
            
        self.goal_prob = goal_prob
        self.final_path = []
        self.min_dis = min_dis
        self.max_dis = max_dis
        
        self.length = length
        self.waypoint = waypoint
        self.node_list = [self.start_point]
        
        self.final_path = []
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.waypoint_pub = rospy.Publisher("/waypoints", String, queue_size=5)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

        
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
            
            nearest_node_index = self.nearest_node_finder(self.node_list, random_node)
            nearest_node = self.Node(self.node_list[nearest_node_index].x, self.node_list[nearest_node_index].y)
            self.chain(nearest_node=nearest_node, random_node=random_node)
            
            if self.obstacle_collision_detector(random_node):
                return
            
            self.node_list.append(random_node)            
            if self.goal_reach(random_node):
                print("Goal Reached")
                flag = 1
                self.final_path = self.generate_goal_path()
                print("Before: path check ",len(self.final_path))
                self.chain(self.start_point, self.final_path[-1])
                self.final_path = self.final_course()
                
                for i in self.final_path:
                    self.path_spawner(i.x, i.y)
                    print(i.x, i.y)
                return

    def final_course(self):
        path = self.final_path
        for node in self.final_path:

            if node.dist > self.length:
                path1 = self.final_path[:self.final_path.index(node)]
                path2 = self.final_path[self.final_path.index(node):]
            
                new_node = self.Node(node.x, node.y)
                new_node.dist = node.dist
                new_node.theta = node.theta
                new_node.parent_node = node.parent_node
                num_of_waypoints = int(math.floor(node.dist / self.length))
                
                for i in range(0, num_of_waypoints):
                    new_node.x -=  self.length* math.cos(node.theta)
                    new_node.y -= self.length * math.sin(node.theta)
                    temp_node = self.Node(new_node.x, new_node.y)
                    if self.obstacle_collision_detector(new_node):
                        temp_node.y -= 4 * math.sin(node.theta) 
                        
                    new_node.path.append(temp_node)
                path1.extend(new_node.path)
                path1.extend(path2)

                path1.sort(key= lambda node: node.x)
                path = path1
                for i in path:
                    print(i.x, i.y)
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

    @staticmethod
    def calculate_parameter(node_initial, node_final):
        dx = node_initial.x - node_final.x
        dy = node_initial.y - node_final.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return dist, theta
   
    def random_node_generator(self):
        if random.randrange(0, 100) >= self.goal_prob:
            
            x_node = random.uniform(self.min_dis, self.max_dis)
            y_node = random.uniform(self.min_dis, self.max_dis)
            
            self.node_spawner(locx=x_node, locy= y_node)
        # write the else part
            return self.Node(x_node, y_node)
    
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



if __name__ == "__main__":
#     latitude: 49.9000764509 lat_to_x(49.900073224), long_to_y(8.8999880161)
# longitude: 8.89997458738
    
    count = 0
    rospy.init_node("rrt_node_python")

    obstacle_x = RRT_Planner.lat_to_x(49.900073224)
    obstacle_y = RRT_Planner.long_to_y(8.8999880161)
    obstacle_loc = [(obstacle_x, obstacle_y, 2)]
    max_iter = 300
    obj = RRT_Planner(goal_prob= 0, min_dis= -10, max_dis=10, obstacle_loc=obstacle_loc, start_point=[0, 0], end_point= [8, 8], length= 2, waypoint= 1, goal_radius=1.5)
    
    while not rospy.is_shutdown() and count < max_iter:
        obj.rtt_planner()
        count += 1