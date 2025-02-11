#include "ros_debug/protobuf_rtps/sensor_data_service_impl.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <functional>
#include <arpa/inet.h>
#include <unistd.h>

#include "chassis/chassis_status.h"
#include "chassis/chassis.h"
#include "ros_debug/proto/sensor_data.pb.h"
#include "get_out_of_trouble/trouble_common.h"
#include "AlongWall/handle_line_laser.h"

#include "log.h"
#define TAG "ROS_DEBUG"

namespace ros_debug
{
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd y(yaw, Eigen::Vector3d::UnitZ());
 
    Eigen::Quaterniond q = y * p * r;
    return q;
 
}

float eslapedtimeSecond(const std::chrono::time_point<std::chrono::system_clock>& start, const std::chrono::time_point<std::chrono::system_clock>& end)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 0.001;
}

SensorDataServiceImpl::SensorDataServiceImpl() : m_thrd_shutdown(false)
{
    
}

SensorDataServiceImpl::~SensorDataServiceImpl()
{
    m_thrd_shutdown = true;
    if(m_thrd && m_thrd->joinable())
    {
        m_thrd->join();
        m_thrd.reset();
    }

    if(m_listen_thrd && m_listen_thrd->joinable())
    {
        m_listen_thrd->join();
        m_listen_thrd.reset();
    }
}

SensorDataServiceImpl& SensorDataServiceImpl::GetInstance()
{
    static SensorDataServiceImpl instance;
    return instance;
}

bool SensorDataServiceImpl::init()
{
    socklen_t clientLen = sizeof(m_client_addr);
    
    // 创建TCP套接字
    m_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_server_socket < 0) {
        LOGE("Error creating socket.");
        return false;
    }
    
    // 设置端口复用
    int opt = 1;
    setsockopt(m_server_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

    m_server_addr.sin_family = AF_INET;
    // m_server_addr.sin_addr.s_addr = INADDR_ANY;
    m_server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    m_server_addr.sin_port = htons(PORT);
    
    int cnt = 0;
    bool bind_ok = false;
    while(cnt++ < 1000)
    {
        // 绑定套接字
        if (bind(m_server_socket, (struct sockaddr*)&m_server_addr, sizeof(m_server_addr)) < 0) {
            LOGE("Binding failed, try again");
            usleep(300000);
            continue;
        }
        else
        {
            bind_ok = true;
            break;
        }
    }
    if(!bind_ok)
    {
        return false;
    }

    if (listen(m_server_socket, 1) < 0) {
        LOGE("----Listen failed.");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return false;
    }

    m_thrd = std::make_unique<std::thread>(std::bind(&SensorDataServiceImpl::run, this));
    // m_listen_thrd = std::make_unique<std::thread>(std::bind(&SensorDataServiceImpl::waitClient, this));

    m_last_laser_scan_time = std::chrono::system_clock::now();
    m_last_line_laser_time = std::chrono::system_clock::now();
    m_last_odom_time = std::chrono::system_clock::now();

    LOGD("init success.");
    return true;
}

void SensorDataServiceImpl::waitClient()
{
    while(!m_thrd_shutdown)
    {
        LOGD("Server listening on port: %d", PORT);
        // 监听
        if (listen(m_server_socket, 1) < 0) {
            LOGE("----Listen failed.");
            m_client_socket = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        // 等待客户端连接
        socklen_t clientLen = sizeof(m_client_addr);
        m_client_socket = accept(m_server_socket, (struct sockaddr*)&m_client_addr, &clientLen);
        if (m_client_socket < 0) {
            LOGE("----Error accepting connection.");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
    }
    LOGD("----wait client");
}

void SensorDataServiceImpl::run()
{
    // char buffer[1024];
    while(!m_thrd_shutdown)
    {
        // 等待客户端连接
        socklen_t clientLen = sizeof(m_client_addr);
        m_client_socket = accept(m_server_socket, (struct sockaddr*)&m_client_addr, &clientLen);
        if (m_client_socket < 0) {
            LOGE("----Error accepting connection.");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        LOGD("Client connected. Start sending data...");

        while(!m_thrd_shutdown)
        {
            std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
            // if(!TROUBLE::CTroubleCommon::getInstance().isConditionMode())
            // {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //     continue;
            // }

            if(m_client_socket < 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                break;
            }

            // memset(buffer, 0, sizeof(buffer));
            // int bytes_received = recv(m_client_socket, buffer, sizeof(buffer), 0);
            // if (bytes_received <= 0) {
            //     LOGE("----Error receiving data.");
            //     break;
            // }

            if(!sendData())
            {
                break;
            }

            std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
            float dt_ms = eslapedtimeSecond(t1, t2) * 1000.0f;
            float sleep_ms = 20.0f - dt_ms;
            if(sleep_ms > 0.0f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds((int)sleep_ms));
            }
            else
            {
                LOGW("----sleep_ms: %f", sleep_ms);
            }
        }

        close(m_client_socket);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    close(m_client_socket);
}

bool SensorDataServiceImpl::sendData() 
{
    // 清空数据
    m_sensor_data.Clear();
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    m_sensor_data.set_secs(ts.tv_sec);
    m_sensor_data.set_nsecs(ts.tv_nsec);
    LaserScanToProto();
    rawLineLaerToProto();
    rawOdomToProto();
    // 序列化数据
    std::string serializedData;
    if (!m_sensor_data.SerializeToString(&serializedData)) {
        std::cerr << "Failed to serialize data." << std::endl;
        return true;
    }
    
    // 发送数据
    int dataSize = serializedData.size();
    if (send(m_client_socket, serializedData.c_str(), dataSize, 0) <= 0) {
        LOGE("--Error sending data!!");
        return false;
    }

    return true;
}

void SensorDataServiceImpl::LaserScanToProto()
{
    m_sensor_data.mutable_laser_scan_360()->Clear();
    m_sensor_data.mutable_raw_laser_scan()->Clear();
    float dt = eslapedtimeSecond(m_last_laser_scan_time, std::chrono::system_clock::now());
    if(dt < 1.0 / LADAR_FREQUENCY)
    {
        return;
    }
    m_last_laser_scan_time = std::chrono::system_clock::now();

    // 360束的激光雷达数据
    LaserInfo laser_info_360;
    LASER::CLaserDataProcess::GetInstance().RestoreLaserData(laser_info_360);
    // 原始的激光雷达数据
    LaserInfo raw_laser_info;
    LASER::CLaserDataProcess::GetInstance().GetOriUnDistortionLaserInfo(raw_laser_info);

    int32_t n = laser_info_360.mLaserData.size();
    // LOGD("laser_info_360.mLaserData.size(): %d", n);
    if(n > 0)
    {
        PointCloud* scan_360 = m_sensor_data.mutable_laser_scan_360();
        if(!LaserToPointCloud(laser_info_360, scan_360))
        {
            scan_360->Clear();
        }
    }

    n = raw_laser_info.mLaserData.size();
    // LOGD("raw_laser_info.mLaserData.size(): %d", n);
    // 打印原始激光数据
    if(n > 0)
    {
        PointCloud* raw_scan = m_sensor_data.mutable_raw_laser_scan();
        if(!LaserToPointCloud(raw_laser_info, raw_scan))
        {
            raw_scan->Clear();
        }
    }
}
void SensorDataServiceImpl::rawLineLaerToProto()
{
    m_sensor_data.mutable_raw_point_cloud()->Clear();
    m_sensor_data.mutable_obs_point_cloud()->Clear();
    float dt = eslapedtimeSecond(m_last_line_laser_time, std::chrono::system_clock::now());
    if(dt < 1.0 / LINE_LASER_FREQUENCY)
    {
        return;
    }
    m_last_line_laser_time = std::chrono::system_clock::now();
    
    // 原始线激光数据
    std::vector<LineLaser::LineLaserData> lineLaser;
    LineLaser::get_linelaser_datas(lineLaser); // 包含最新的20帧数据
    // LOGD("lineLaser size: %d, empty: %d", lineLaser.size(), lineLaser.empty());
    // 避障框数据
    std::vector<std::vector<cv::Point3f>> obs_line_laser;
    LineLaser::get_obstacles_data(obs_line_laser);
    LOGD("obs_line_laser size: %d", obs_line_laser.size());
    if(!lineLaser.empty())
    {
        // 取最后2侦
        PointCloud* raw_points = m_sensor_data.mutable_raw_point_cloud();
        for(int i = lineLaser.size() - 1; (i >= (lineLaser.size() - 2)) && (i >= 0); i--)
        {
            const LineLaser::LineLaserData& raw_line_laser_data = lineLaser[i];
            // LOGD("raw_line_laser_data size: %d", raw_line_laser_data.line_laser.size());

            // 先激光坐标系原点在机器人坐标系下的坐标
            float to_center_x, to_center_y;
            // 扫地机的坐标系为前x，左y
            if (raw_line_laser_data.addr == 1)  // 水平线激光
            {
                // 前方水平线激光的IR相机在扫地机坐标系的坐标点
                to_center_x = 170.3f;
                to_center_y = 0.0f;
            }
            else if (raw_line_laser_data.addr == 2)  // 垂直线激光
            {
                to_center_x = 48.72f;
                to_center_y = -162.4f;
            }

            if(!raw_line_laser_data.line_laser.empty())
            {
                for(const auto & raw_data : raw_line_laser_data.line_laser)
                {
                    if(std::isnan(raw_data.x) || std::isnan(raw_data.y) || std::isnan(raw_data.z))
                    {
                        continue;
                    }
                
                    if(std::fabs(raw_data.x) < 1.0f && std::fabs(raw_data.y) < 1.0f)
                    {
                        continue;
                    }
                    // 转换到机器人坐标系
                    const float robot_px = raw_data.y + to_center_x;
                    const float robot_py = -raw_data.x + to_center_y;
                    YXPoint3D* proto_point = raw_points->add_points();
                    proto_point->set_x(robot_px * 10.0);
                    proto_point->set_y(robot_py * 10.0);
                    proto_point->set_z(0);
                }
            }

            // LOGD("proto raw_points: %d", m_sensor_data.raw_point_cloud().points_size());
        }   
    }

    // 避障框
    if(obs_line_laser.size())
    {
        // LOGD("obs_data: %d", obs_line_laser.front().size());
        PointCloud* obs_points = m_sensor_data.mutable_obs_point_cloud();
        for(const auto & obs_data : obs_line_laser)
        {
            for(const auto & obs_point : obs_data)
            {
                if(std::isnan(obs_point.x) || std::isnan(obs_point.y) || std::isnan(obs_point.z))
                {
                    continue;
                }

                if(std::fabs(obs_point.x) < 1.0f && std::fabs(obs_point.y) < 1.0f)
                {
                    continue;
                } 
                YXPoint3D* proto_point = obs_points->add_points();
                proto_point->set_x(obs_point.x * 10.0);
                proto_point->set_y(obs_point.y * 10.0);
                proto_point->set_z(0);
            }
        }

        // LOGD("proto obs_points: %d", m_sensor_data.obs_point_cloud().points_size());
    }
}

void SensorDataServiceImpl::rawOdomToProto()
{
    m_sensor_data.mutable_odometry()->Clear();
    float dt = eslapedtimeSecond(m_last_odom_time, std::chrono::system_clock::now());
    if(dt < 1.0 / ODOM_FREQUENCY)
    {
        return;
    }
    m_last_odom_time = std::chrono::system_clock::now();
    
    PROTOCOL::EncoderData odom_pose = CHASSIS::CChassis::GetInstance().Status().GetEncoderData();
    
    Odometry_Pose* odom_pose_proto = m_sensor_data.mutable_odometry()->mutable_pose();
    Odometry_Pose_Position* odom_position_proto = odom_pose_proto->mutable_position();
    odom_position_proto->set_x(odom_pose.x * 1000.0);
    odom_position_proto->set_y(odom_pose.y * 1000.0);
    odom_position_proto->set_z(0.0);
    
    // 欧拉角转四元数
    Odometry_Pose_Orientation* odom_orientation_proto = odom_pose_proto->mutable_orientation();
    /// 里程计角度为[0,2pi]，转到[-pi, pi]
    if(odom_pose.th > M_PI)
    {
        odom_pose.th -= 2.0 * M_PI;
    }
    const auto q = euler2Quaternion(0.0, 0.0, odom_pose.th);
    odom_orientation_proto->set_x(q.x() * 1000.0);
    odom_orientation_proto->set_y(q.y() * 1000.0);
    odom_orientation_proto->set_z(q.z() * 1000.0);
    odom_orientation_proto->set_w(q.w() * 1000.0);

    float left_wheel_speed, right_wheel_speed;
    CHASSIS::CChassis::GetInstance().Status().GetWheelSpeed(left_wheel_speed, right_wheel_speed);

    // 将轮速转换为机器人中心速度
    float linear_speed = (left_wheel_speed + right_wheel_speed) / 2.0;
    float angular_speed = (right_wheel_speed - left_wheel_speed) / DISTANCE_BETWEEN_WHEELS;
    Odometry_Twist* odom_twist = m_sensor_data.mutable_odometry()->mutable_twist();
    Odometry_Twist_Linear* odom_linear = odom_twist->mutable_linear();
    odom_linear->set_x(linear_speed * 1000.0);
    odom_linear->set_y(0.0);
    odom_linear->set_z(0.0);
    Odometry_Twist_Angular* odom_angular = odom_twist->mutable_angular();
    odom_angular->set_x(0.0);
    odom_angular->set_y(0.0);
    odom_angular->set_z(angular_speed * 1000.0);
}

bool SensorDataServiceImpl::LaserToPointCloud(const LaserInfo& laser_info, PointCloud* point_cloud)
{
    if(laser_info.mLaserData.empty() || point_cloud == nullptr)
    {
        return false;
    }
    
    const float degree2radian = M_PI / 180.0f;
    auto proto_points = point_cloud->mutable_points();
    for(const auto & laser : laser_info.mLaserData)
    {
        if(std::isnan(laser.nAngle) || std::isnan(laser.nDist_MM) || laser.nDist_MM < 1.0f)
        {
            continue;
        }
        // 激光雷达顺时针角度，便于后面进行极坐标转笛卡尔坐标，激光雷达x轴朝机器人前方，y朝左
        const float dist = laser.nDist_MM * 0.001f;
        const float laser_px = dist * sin(laser.nAngle * degree2radian);
        const float laser_py = dist * cos(laser.nAngle * degree2radian);
        if(std::fabs(laser_px * 1000.0) < 1.0f && std::fabs(laser_py * 1000.0) < 1.0f)
        {
            continue;
        } 
        YXPoint3D* proto_point = proto_points->Add(); // 精确到毫米后一个小数点
        proto_point->set_x(laser_px * 10000.0);
        proto_point->set_y(laser_py * 10000.0);
        proto_point->set_z(0);
    }

    return true;
}

} // namespace ros_debug

