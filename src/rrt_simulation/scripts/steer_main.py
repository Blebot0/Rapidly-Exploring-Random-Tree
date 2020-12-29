#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range
from tf.transformations import euler_from_quaternion
from pyproj import Geod
import numpy as np
import math
import time
from std_msgs.msg import String

class Steer:

    def __init__(self):
        
        rospy.init_node('steer_node', disable_signals=True)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/waypoints", String, self.waypoint_callback)
        speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.waypoints = ""    
        speed = Twist()

    def waypoint_callback(data):

        pass

    def imu_callback(pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw= math.degrees(euler[2]) +180
        yaw = abs(yaw-360)
        yaw = yaw%360

    def Steer():
        pass



if __name__=="__main__":
    
    obj = Steer()
    while not rospy.is_shutdown():
        geodesic =Geod(ellps='WGS84')
        # bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
        bearing = bearing 
        # angle_diff =bearing-yaw
    

