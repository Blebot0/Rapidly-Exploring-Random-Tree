#include<ros/ros.h>
#include<bits/stdc++.h>
#include<geometry_msgs/Twist.h>
#include<stdlib.h>


int main(int argc, char  **argv)
{

    ros::init(argc, argv, "node_rtt_bot");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist twist_obj;

    ros::Rate loop_rate(30);

    while(ros::ok){
        twist_obj.linear.x = 1;

        pub.publish(twist_obj);
        loop_rate.sleep();
    }

    return 0;
}
