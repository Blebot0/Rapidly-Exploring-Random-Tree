#!/usr/bin/env python

import rospy, tf
import numpy as np
import time
from geometry_msgs.msg import *
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
            self.parent_node = None
            self.path_x = []
            self.path_y = []


    def __init__(self, start_point = None, end_point = None, obstacle_loc = None, goal_prob = None, min_dis = None, max_dis = None, length = None, waypoint = None, goal_radius = None):
        
        self.start_point = self.Node(start_point[0], start_point[1])
        self.end_point = self.Node(end_point[0], end_point[1])
        self.goal_area = (end_point[0], end_point[1], goal_radius)
        self.obstacle_loc = obstacle_loc
        
        self.max_iter = max_iter
        
        self.goal_prob = goal_prob
        self.final_path = []
        self.min_dis = min_dis
        self.max_dis = max_dis
        
        self.length = length
        self.waypoint = waypoint
        self.node_list = [self.start_point]
        
        
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        
        self.num = 1
    
    def node_spawner(self, locx, locy):
        with open("/home/blebot/mowito_ws/src/rrt_simulation/urdf/marker.urdf", "r") as f:
            product_xml = f.read()
        item_name = "marker_{0}".format(self.num)
        print("Spawning model:%s", item_name)
        item_pose   =  Pose(Point(x=locx, y=locy, z=0), Quaternion(0, 0, 0, 0))
        self.spawn_model(item_name, product_xml, "/marker_{}".format(self.num), item_pose, "world")
        self.num += 1

    def rtt_planner(self):
        global flag
        if flag == 0:
            
            random_node = self.random_node_generator()
            
            nearest_node_index = self.nearest_node_finder(self.node_list, random_node)
            nearest_node = self.node_list[nearest_node_index]
            new_node = self.chain(nearest_node=nearest_node, random_node=random_node, length=self.length)
            
            if self.obstacle_collision_detector(new_node):
                return
            
            self.node_list.append(new_node)            
            if self.goal_reach(random_node):
                print("Goal Reached")
                flag = 1
                self.final_path = self.generate_goal_path()
                self.final_path = self.final_path[1:]
                print(self.final_path)
                return


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
        # end node -> parent node -> parent node --- till start node
        path = [[self.end_point.x, self.end_point.y]]
        node = self.node_list[-1]
        while node.parent_node is not None:
            path.append([node.x, node.y])
            node = node.parent_node
        path.append([node.x, node.y])

        return path

    def chain(self, nearest_node, random_node, length):
        new_node = self.Node(nearest_node.x, nearest_node.y)

        dist, theta = self.calculate_parameter(new_node, random_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if length > dist:
            length = dist

        waypoint_num = int(math.floor(length / self.waypoint))

        for i in range(waypoint_num):
            new_node.x += self.waypoint * math.cos(theta)
            new_node.y += self.waypoint * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, theta = self.calculate_parameter(new_node, random_node)

        if d <= self.waypoint:
            new_node.path_x.append(random_node.x)
            new_node.path_y.append(random_node.y)
            new_node.x = random_node.x
            new_node.y = random_node.y

        new_node.parent_node = random_node

        return new_node
        
        
    def calculate_parameter(self, node_initial, node_final):
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
        else:
            return
    
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
    obstacle_loc = [(obstacle_x, obstacle_y, 1)]
    max_iter = 300
    obj = RRT_Planner(goal_prob= 0, min_dis= -10, max_dis=10, obstacle_loc=obstacle_loc, start_point=[0, 0], end_point= [8, 8], length= 3, waypoint= 1, goal_radius=1.5)
    
    while not rospy.is_shutdown() or count < max_iter:
        obj.rtt_planner()
        count += 1