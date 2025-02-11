#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <netinet/in.h>
#include <unistd.h>
#include "sensor_data.pb.h"

class CSocketClient
{
public:
    CSocketClient(const std::string& server_ip, const int m_port);
    ~CSocketClient();
    // bool init();
    bool getData(SensorData& data) {
        std::lock_guard<std::mutex> l(m_mutex);
        if(m_sensor_data.ByteSizeLong() <= 0)
        {
            return false;
        }
        data.CopyFrom(m_sensor_data);

        return true;
    }

private:
    void recvData();

    std::thread* m_thrd;
    std::mutex m_mutex;
    int m_sock;
    struct sockaddr_in m_server_addr;
    SensorData m_sensor_data;
    bool m_thrd_shutdown;
    std::string m_server_ip;
    int m_port;
};