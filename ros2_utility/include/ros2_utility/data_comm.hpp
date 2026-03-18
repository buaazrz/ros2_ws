#pragma once

#include <vector>
#include <string>
#include <memory>
#include <cstring>
#include <iostream>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define MAX_CHANNEL 8
#define MAX_DOF 16

// =========================
// 数据结构
// =========================
struct RobotData
{
    double t{0.0};
    int channel_size[MAX_CHANNEL]{0};
    double data[MAX_CHANNEL][MAX_DOF]{0};
};

// =========================
// log2Channel 工具函数
// =========================
inline void log2Channel(RobotData &rd, int ch, const double *src, int len)
{
    if (ch >= MAX_CHANNEL) return;
    rd.channel_size[ch] = len;
    std::memcpy(rd.data[ch], src, sizeof(double) * len);
}

// =========================
// UDP通信类（单例）
// =========================
class DataComm
{
public:
    static DataComm* getInstance()
    {
        static DataComm instance;
        return &instance;
    }

    void setDestAddress(const std::string& ip, int port)
    {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);

        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr);

        std::cout << "[DataComm] send to " << ip << ":" << port << std::endl;
    }

    void sendRobotStatus(const RobotData &data)
    {
        if (sock_ < 0) return;

        sendto(sock_,
               (const char*)&data,
               sizeof(RobotData),
               0,
               (struct sockaddr*)&addr_,
               sizeof(addr_));
    }

private:
    DataComm() {}
    ~DataComm()
    {
        if (sock_ > 0) close(sock_);
    }

private:
    int sock_{-1};
    struct sockaddr_in addr_;
};