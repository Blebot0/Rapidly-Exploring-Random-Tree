#!/usr/bin/env python

from numpy.core.fromnumeric import amax
from numpy.lib.function_base import angle
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range
from tf.transformations import euler_from_quaternion
from rrt_simulation.msg import Waypoint
import numpy as np
import math
import time
from std_msgs.msg import *
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

class Steer:

    def __init__(self):
        
        rospy.init_node('steer_node', disable_signals=True)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/waypoints", Waypoint, self.waypoint_callback)
        rospy.Subscriber("/waypoint_bool", Bool, self.waypoint_bool_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.waypoints = []  
        self.speed = Twist()
        self.waypoint_bool = False
        self.waypoint_num = 0
        self.start_point = [0, 0]
        self.yaw = 0
        self.x = 0
        self.y = 0

    def state_callback(self, data):
        self.x = data.pose[1].position.x
        self.y = data.pose[1].position.y

    def waypoint_bool_callback(self, data):
        self.waypoint_bool = data.data

    def waypoint_callback(self, data):
        self.waypoints.append([data.x_waypoint, data.y_waypoint])

    def imu_callback(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw= math.degrees(euler[2]) + 360
        self.yaw = self.yaw%360

    def calculate_parmaters(self):
        dx = self.waypoints[self.waypoint_num][0] - self.x
        dy = self.waypoints[self.waypoint_num][1] - self.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        theta = math.degrees(theta) + 360
        theta = theta % 360

        return dist, theta
    
    def Steer(self):
        if self.waypoint_bool == True and self.waypoint_num != len(self.waypoints):
            dist, theta = self.calculate_parmaters()
            
            while True:
                print(self.waypoints[self.waypoint_num])
                angle_diff = self.yaw - theta
                print(angle_diff)
                if angle_diff < 1 and angle_diff >-1:
                    dist, _ = self.calculate_parmaters()
                    if dist < 0.5:
                        self.waypoint_num += 1
                        break
                    elif dist >=0.1:
                        self.speed.angular.z = 0
                        self.speed.linear.x = 0.5
                        self.speed_pub.publish(self.speed)

                # elif angle_diff < -180:
                #     self.speed.angular.z = -1.
                #     self.speed.linear.x = 0
                #     self.speed_pub.publish(self.speed)

                elif angle_diff>=0 and angle_diff <= 180:
                    self.speed.angular.z = 1.
                    self.speed.linear.x = 0
                    self.speed_pub.publish(self.speed)
                
                elif angle_diff<0 or angle_diff > 180:
                    self.speed.angular.z = -1.
                    self.speed.linear.x = 0
                    self.speed_pub.publish(self.speed)

        else:
            self.speed.angular.z = 0
            self.speed.linear.x = 0
            self.speed_pub.publish(self.speed)

            
            




    @staticmethod
    def long_to_y(input_longitude):
        return -105292.0089353767 * (input_longitude - 8.90003)
    
    @staticmethod
    def lat_to_x(input_latitude):
        return 110692.0702932625 * (input_latitude - 49.90002809)

if __name__=="__main__":
    
    obj = Steer()
    while not rospy.is_shutdown():
        obj.Steer()
        

