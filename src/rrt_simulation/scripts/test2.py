#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from dynamic_reconfigure.srv import Reconfigure
if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("/fix/position/set_parameters")
    print("Got it.")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    gps_service = rospy.ServiceProxy("/fix/position/set_parameters", Reconfigure)
    
    with open("/home/blebot/mowito_ws/src/rrt_simulation/urdf/marker_black.urdf", "r") as f:
        product_xml = f.read()



    for num in range(0,115):
        bin_y   =   2.8 *   (num    /   6)  -   1.4 
        bin_x   =   0.5 *   (num    %   6)  -   1.5
        item_name   =   "product_{0}_0".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y, z=0), Quaternion(0, 0, 0, 0))
        spawn_model(item_name, product_xml, "/marker_", item_pose, "world")
