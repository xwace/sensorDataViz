#include "debug_robot/tcp_socket_client.hpp"

#include <iostream>
#include <chrono>
#include <arpa/inet.h>
#include <unistd.h>

CSocketClient::CSocketClient(const std::string& server_ip, const int m_port) 
: m_thrd(nullptr), 
m_thrd_shutdown(false),
m_server_ip(server_ip),
m_port(m_port)
{
    m_thrd = new std::thread(&CSocketClient::recvData, this);
}

CSocketClient::~CSocketClient()
{ 
    m_thrd_shutdown = true;
    if(m_thrd && m_thrd->joinable())
    {
        m_thrd->join();
        delete m_thrd;
    }

    close(m_sock);
}

/*
bool CSocketClient::init()
{
    // 创建套接字
    m_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (m_sock < 0) {
        std::cerr << "Error creating socket." << std::endl;
        return false;
    }

    m_server_addr.sin_family = AF_INET;
    m_server_addr.sin_port = htons(PORT);
    m_server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    
    // 连接到服务端
    if (connect(m_sock, (struct sockaddr*)&m_server_addr, sizeof(m_server_addr)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        return false;
    }

    std::cout << "Connected to server." << std::endl;
    m_thrd = new std::thread(&CSocketClient::recvData, this);
    std::cout << "init success" << std::endl;

    return true;
}
*/
/*
void CSocketClient::recvData()
{
    char buffer[8192] = {0};
    std::string send_msg = "hello\n";
    while (!m_thrd_shutdown)
    {
        std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

        send(m_sock, send_msg.c_str(), send_msg.size(), 0);
        memset(buffer, 0, sizeof(buffer));
        int dataSize = recv(m_sock, buffer, sizeof(buffer), 0);
        // 接收实际数据
        if (dataSize <= 0) {
            std::cerr << "Error receiving data" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::cout << "dataSize: " << dataSize << std::endl;

        // 反序列化数据
        {
            std::lock_guard<std::mutex> l(m_mutex);
            m_sensor_data.Clear();
            if (!m_sensor_data.ParseFromArray(buffer, dataSize)) {
                // std::cerr << "Failed to parse data: " << m_sensor_data.DebugString() << std::endl;
            }
            // std::cout << "odom: " << m_sensor_data.odometry().ByteSizeLong() << std::endl;
            // std::cout << "raw scan: " << m_sensor_data.raw_laser_scan().ByteSizeLong() << std::endl;
            // std::cout << "360 scan: " << m_sensor_data.laser_scan_360().ByteSizeLong() << std::endl;
            std::cout << "raw pointcloud: " << m_sensor_data.raw_point_cloud().points_size() << std::endl;
            std::cout << "obs pointcloud: " << m_sensor_data.obs_point_cloud().points_size() << std::endl;
        }
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        float dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        float sleep_ms = 20.0f - dt_ms;
        if(sleep_ms > 0.0f)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds((int)sleep_ms));
        }
        else
        {
            std::cout << "----sleep_ms: " << sleep_ms << std::endl;
        }
    }
    std::cout << "recvData end" << std::endl;
}
*/

void CSocketClient::recvData()
{
    char buffer[8192] = {0};
    while(!m_thrd_shutdown)
    {
        // 创建套接字
        m_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (m_sock < 0) {
            std::cerr << "Error creating socket." << std::endl;
            return;
        }

        m_server_addr.sin_family = AF_INET;
        m_server_addr.sin_port = htons(m_port);
        m_server_addr.sin_addr.s_addr = inet_addr(m_server_ip.c_str());
        // inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
        
        std::cout << "Connecting to server [" << m_server_ip << ":" << m_port << "] ..." << std::endl;
        // 连接到服务端
        if (connect(m_sock, (struct sockaddr*)&m_server_addr, sizeof(m_server_addr)) < 0) {
            std::cerr << "Connection failed. Retrying..." << std::endl;
            close(m_sock);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        std::cout << "Connected to server. Start receiving data..." << std::endl;

        while (!m_thrd_shutdown)
        {
            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            memset(buffer, 0, sizeof(buffer));
            int dataSize = recv(m_sock, buffer, sizeof(buffer), 0);
            // 接收实际数据
            if (dataSize <= 0) {
                std::cerr << "Error receiving data" << std::endl;
                break;
            }
            // std::cout << "dataSize: " << dataSize << std::endl;

            // 反序列化数据
            {
                std::lock_guard<std::mutex> l(m_mutex);
                m_sensor_data.Clear();
                if (!m_sensor_data.ParseFromArray(buffer, dataSize)) {
                    // std::cerr << "Failed to parse data: " << m_sensor_data.DebugString() << std::endl;
                }
                // std::cout << "odom: " << m_sensor_data.odometry().ByteSizeLong() << std::endl;
                // std::cout << "raw scan: " << m_sensor_data.raw_laser_scan().ByteSizeLong() << std::endl;
                // std::cout << "360 scan: " << m_sensor_data.laser_scan_360().ByteSizeLong() << std::endl;
                // std::cout << "raw pointcloud: " << m_sensor_data.raw_point_cloud().points_size() << std::endl;
                // std::cout << "obs pointcloud: " << m_sensor_data.obs_point_cloud().points_size() << std::endl;
            }
            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            float dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            float sleep_ms = 30.0f - dt_ms;
            if(sleep_ms > 0.0f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds((int)sleep_ms));
            }
            else
            {
                std::cout << "----sleep_ms: " << sleep_ms << std::endl;
            }
        }

        close(m_sock);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "recvData end" << std::endl;
}