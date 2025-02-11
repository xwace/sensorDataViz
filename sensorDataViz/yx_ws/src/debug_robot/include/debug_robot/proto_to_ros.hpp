#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <thread>

#include "debug_robot/tcp_socket_client.hpp"
#include "sensor_data.pb.h"

#include <ros/ros.h>
#include <ros/time.h>

ros::Time convertToRosTime(int64_t timestamp);

class ProtoToRos
{
public:
    ProtoToRos(const ros::NodeHandle& nh);
    ~ProtoToRos();
    bool init();
    void publish();

private:
    void protoOdometryToRos(const Odometry& proto_odom, const ros::Time& timestamp, nav_msgs::Odometry& ros_odom);
    void protoLaserScanToRos(const LaserScan& proto_laser_scan, const ros::Time& timestamp, sensor_msgs::LaserScan& ros_laser_scan);
    void protoPointCloudToRos(const PointCloud& proto_point_cloud, const ros::Time& timestamp, const std::string&frame_id, sensor_msgs::PointCloud2& ros_point_cloud);
    sensor_msgs::PointCloud2 convertToPointCloud2(const PointCloud& yx_point_cloud);

    ros::NodeHandle m_nh;
    ros::Publisher m_odom_pub;
    ros::Publisher m_laser_scan_360_pub, m_raw_laser_scan_pub;
    ros::Publisher m_raw_point_cloud_pub, m_obs_point_cloud_pub;
    tf::TransformBroadcaster m_odom_tf_broadcaster;
    tf::TransformBroadcaster m_laser_tf_broadcaster;
    std::unique_ptr<CSocketClient> m_socket_client;
};