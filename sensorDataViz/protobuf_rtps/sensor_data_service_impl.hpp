#pragma once

#include "chassis/chassis_status.h"
#include "ros_debug/proto/sensor_data.pb.h"
#include <thread>
#include <memory>
#include <atomic>
#include <netinet/in.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ros_debug
{

#define PORT 6558
#define SERVER_IP "10.10.35.228"
// #define PORT 8081
// #define SERVER_IP "172.16.4.10"

// 发布频率
#define LADAR_FREQUENCY 10
#define LINE_LASER_FREQUENCY 10
#define ODOM_FREQUENCY 50

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);
float eslapedtimeSecond(const std::chrono::time_point<std::chrono::system_clock>& start, const std::chrono::time_point<std::chrono::system_clock>& end);

class SensorDataServiceImpl final
{
public:
    ~SensorDataServiceImpl();
    static SensorDataServiceImpl& GetInstance();
    bool init();
private:
    SensorDataServiceImpl();
    void run();
    void waitClient();
    bool sendData();
    void LaserScanToProto();
    void rawLineLaerToProto();
    void rawOdomToProto();
    bool LaserToPointCloud(const LaserInfo& laser_info, PointCloud* point_cloud);

    bool m_thrd_shutdown;
    std::unique_ptr<std::thread> m_thrd, m_listen_thrd;
    int m_server_socket;
    int m_client_socket;
    struct sockaddr_in m_server_addr, m_client_addr;
    SensorData m_sensor_data;
    std::atomic_bool m_client_connected;
    std::chrono::time_point<std::chrono::system_clock> m_last_laser_scan_time;
    std::chrono::time_point<std::chrono::system_clock> m_last_line_laser_time;
    std::chrono::time_point<std::chrono::system_clock> m_last_odom_time;
};

} // end namespace ros_debug

