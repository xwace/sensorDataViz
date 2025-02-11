#include "debug_robot/proto_to_ros.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Time convertToRosTime(int64_t timestamp) {
    // 假设时间戳是毫秒
    int sec = timestamp / 1000;  // 获取秒数
    int nsec = (timestamp % 1000) * 1000000;  // 获取纳秒数
    return ros::Time(sec, nsec);
}

ProtoToRos::ProtoToRos(const ros::NodeHandle& nh) : m_nh(nh)
{
    std::string server_ip;
    int port;

    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("server_ip", server_ip, "127.0.0.1");
    private_nh.param<int>("port", port, 6558);
    std::cout << "server_ip: " << server_ip << ", port: " << port << std::endl;
    m_socket_client.reset(new CSocketClient(server_ip, port));
}

ProtoToRos::~ProtoToRos()
{

}

bool ProtoToRos::init()
{
    m_odom_pub = m_nh.advertise<nav_msgs::Odometry>("odom", 10);
    m_laser_scan_360_pub = m_nh.advertise<sensor_msgs::PointCloud2>("scan_360", 10);
    m_raw_laser_scan_pub = m_nh.advertise<sensor_msgs::PointCloud2>("raw_scan", 10);
    m_raw_point_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("raw_point_cloud", 10);
    m_obs_point_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("obs_point_cloud", 10);
    return true;
}

void ProtoToRos::publish()
{
    nav_msgs::Odometry odom_msg;
    sensor_msgs::PointCloud2 laser_scan_360_msg, raw_laser_scan_msg;
    sensor_msgs::PointCloud2 raw_point_cloud_msg, obs_point_cloud_msg;
    SensorData proto_data;
    if(m_socket_client && !m_socket_client->getData(proto_data))
    {
        return;
    }

    // ros::Time timestamp(proto_data.secs(), proto_data.nsecs());
    // static ros::Time last_time_stamp(0, 0);
    // if(timestamp == last_time_stamp)
    // {
    //     return;
    // }
    // last_time_stamp = timestamp;
    ros::Time timestamp = ros::Time::now();
    static tf::Quaternion odom_q(0.0, 0.0, 0.0, 1.0);
    static tf::Transform odom_transform(odom_q, tf::Vector3(0.0, 0.0, 0.0));
    if(proto_data.odometry().ByteSizeLong() > 0)
    {
        protoOdometryToRos(proto_data.odometry(), timestamp, odom_msg);
        m_odom_pub.publish(odom_msg);
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odom_q);
        odom_transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
        odom_transform.setRotation(odom_q);
    }
    
    if(proto_data.laser_scan_360().points_size() > 0)
    {
        protoPointCloudToRos(proto_data.laser_scan_360(), timestamp, "laser", laser_scan_360_msg);
        m_laser_scan_360_pub.publish(laser_scan_360_msg);
    }

    if(proto_data.raw_laser_scan().points_size() > 0)
    {
        protoPointCloudToRos(proto_data.raw_laser_scan(), timestamp, "laser", raw_laser_scan_msg);
        m_raw_laser_scan_pub.publish(raw_laser_scan_msg);
    }

    if(proto_data.raw_point_cloud().points_size() > 0)
    {
        protoPointCloudToRos(proto_data.raw_point_cloud(), timestamp, "base_link", raw_point_cloud_msg);
        m_raw_point_cloud_pub.publish(raw_point_cloud_msg);
    }

    if(proto_data.obs_point_cloud().points_size() > 0)
    {
        protoPointCloudToRos(proto_data.obs_point_cloud(), timestamp, "base_link", obs_point_cloud_msg);
        m_obs_point_cloud_pub.publish(obs_point_cloud_msg);
    }

    // 发布tf
    // static tf::Transform baselink2laser_transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.135, 0.0, 0.05));
    // m_laser_tf_broadcaster.sendTransform(tf::StampedTransform(baselink2laser_transform, timestamp, "base_link", "laser"));
    m_odom_tf_broadcaster.sendTransform(tf::StampedTransform(odom_transform, timestamp, "odom", "base_link"));

}

void ProtoToRos::protoOdometryToRos(const Odometry& proto_odom, const ros::Time& timestamp, nav_msgs::Odometry& ros_odom)
{
    ros_odom.header.stamp = timestamp;
    ros_odom.header.frame_id = "odom";
    ros_odom.child_frame_id = "base_link";
    ros_odom.pose.pose.position.x = proto_odom.pose().position().x() * 0.001f;
    ros_odom.pose.pose.position.y = proto_odom.pose().position().y() * 0.001f;
    ros_odom.pose.pose.position.z = proto_odom.pose().position().z() * 0.001f;
    ros_odom.pose.pose.orientation.x = proto_odom.pose().orientation().x() * 0.001f;
    ros_odom.pose.pose.orientation.y = proto_odom.pose().orientation().y() * 0.001f;
    ros_odom.pose.pose.orientation.z = proto_odom.pose().orientation().z() * 0.001f;
    ros_odom.pose.pose.orientation.w = proto_odom.pose().orientation().w() * 0.001f;
    ros_odom.twist.twist.linear.x = proto_odom.twist().linear().x() * 0.001f;
    ros_odom.twist.twist.linear.y = proto_odom.twist().linear().y() * 0.001f;
    ros_odom.twist.twist.linear.z = proto_odom.twist().linear().z() * 0.001f;
    ros_odom.twist.twist.angular.x = proto_odom.twist().angular().x() * 0.001f;
    ros_odom.twist.twist.angular.y = proto_odom.twist().angular().y() * 0.001f;
    ros_odom.twist.twist.angular.z = proto_odom.twist().angular().z() * 0.001f;
}

void ProtoToRos::protoLaserScanToRos(const LaserScan& proto_laser_scan, const ros::Time& timestamp, sensor_msgs::LaserScan& ros_laser_scan)
{
    ros_laser_scan.header.stamp = timestamp;
    ros_laser_scan.header.frame_id = "laser";
    // ros_laser_scan.angle_min = proto_laser_scan.angle_min() * 0.001f;
    // ros_laser_scan.angle_max = proto_laser_scan.angle_max() * 0.001f;
    ros_laser_scan.angle_increment = proto_laser_scan.angle_increment() * 0.001f;
    ros_laser_scan.angle_min = -M_PI + ros_laser_scan.angle_increment;
    ros_laser_scan.angle_max = M_PI;
    
    ros_laser_scan.time_increment = proto_laser_scan.time_increment() * 0.001f;
    ros_laser_scan.scan_time = proto_laser_scan.scan_time() * 0.001f;
    ros_laser_scan.range_min = proto_laser_scan.range_min() * 0.001f;
    ros_laser_scan.range_max = proto_laser_scan.range_max() * 0.001f;
    ros_laser_scan.ranges.resize(proto_laser_scan.ranges_size());
    for(int i = 0; i < proto_laser_scan.ranges_size(); i++)
    {
        ros_laser_scan.ranges[i] = proto_laser_scan.ranges(i) * 0.0001f;
    }
}

void ProtoToRos::protoPointCloudToRos(const PointCloud& proto_point_cloud, const ros::Time& timestamp, const std::string& frame_id, sensor_msgs::PointCloud2& ros_point_cloud)
{
    // std::cout << "proto_point_cloud size: " << proto_point_cloud.points_size() << std::endl;
    ros_point_cloud = convertToPointCloud2(proto_point_cloud);
    ros_point_cloud.header.stamp = timestamp;
    ros_point_cloud.header.frame_id = (frame_id.empty() ? "base_link" : frame_id);
}

sensor_msgs::PointCloud2 ProtoToRos::convertToPointCloud2(const PointCloud& yx_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    for (const auto& point : yx_point_cloud.points())
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = (double)point.x() * 0.0001f; // 源头单位为mm * 10，所以这里转换为米时需要除以10000
        pcl_point.y = (double)point.y() * 0.0001f;
        pcl_point.z = (double)point.z() * 0.0001f;
        pcl_cloud.points.push_back(pcl_point);
    }
    
    pcl_cloud.width = static_cast<uint32_t>(pcl_cloud.points.size());
    pcl_cloud.height = 1;
    sensor_msgs::PointCloud2 ros_cloud;
    ros_cloud.data.clear();
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    return ros_cloud;
}
