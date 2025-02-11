#include <ros/ros.h>
#include "debug_robot/proto_to_ros.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "yx_robot_debug");
    ros::NodeHandle nh;
    ProtoToRos proto_to_ros(nh);
    if(!proto_to_ros.init())
    {
        std::cout << "init failed" << std::endl;
        return -1;
    }

    ros::Rate rate(30);
    while(ros::ok())
    {
        proto_to_ros.publish();
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}