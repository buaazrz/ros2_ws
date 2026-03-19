#ifndef ROS2_UTILITY_DATA_COMM_HPP
#define ROS2_UTILITY_DATA_COMM_HPP

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <chrono>
#include <cstdio>

using namespace std::chrono;
using namespace std;

// 全局线程运行标志（全功能保留）
static volatile bool loop = true;

// 机器人核心数据结构（与控制器完全匹配）
struct RobotData
{
    double t;
    float q[4][7];
};

// 接收回调函数类型（保留）
typedef void (*ReceiveCallBack)(void *, const RobotData &);

// 线程参数结构体（保留）
struct ClientData
{ 
    ReceiveCallBack fnc;
    int port;
    double dt;
    void *pClientData;
};

// 数据日志写入文件（全功能保留）
inline void log2file(std::ofstream &fout, std::vector<RobotData> &frames)
{
    for (std::size_t k = 0; k < frames.size(); k++)
    {
        fout << frames[k].t << "\n";
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 7; j++)
                fout << frames[k].q[i][j] << (j == 6 ? "\n" : " ");

        fout << "\n";
    }
}

// 接收线程主函数（全功能保留：UDP绑定、超时接收、回调、定时、日志）
inline void *service_func(void *pdata)
{
    int port = reinterpret_cast<ClientData *>(pdata)->port;
    ReceiveCallBack func = reinterpret_cast<ClientData *>(pdata)->fnc;
    double dt = reinterpret_cast<ClientData *>(pdata)->dt;
    void *pClientData = reinterpret_cast<ClientData *>(pdata)->pClientData;
    int socket_handle = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_handle == -1)
    {
        printf("create socket in service failed\n");
        return nullptr;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(socket_handle, (struct sockaddr *)&addr, sizeof(addr)))
    {
        printf("bind socket in service failed\n");
        return nullptr;
    }
    ofstream fout("log.txt");
    fout.setf(std::ios::fixed);
    fout.precision(9);
    RobotData data;
    vector<RobotData> frames;
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    if (setsockopt(socket_handle, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)))
    {
        printf("set socket time out in service failed\n");
        return nullptr;
    }
    while (loop)
    {
        auto t_start = high_resolution_clock::now();
        int resv_num = recvfrom(socket_handle, &data, sizeof(data), 0, nullptr, nullptr);
        if (resv_num > 0)
        {
            if (func)
                func(pClientData, data);
            frames.push_back(data);
        }
        if (dt > 0)
        {
            auto t_stop = high_resolution_clock::now();
            auto t_duration = std::chrono::duration<double>(t_stop - t_start);
            if (t_duration.count() < dt)
                std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
    }
    log2file(fout, frames);
    close(socket_handle);

    printf("stop receiving\n");
    return nullptr;
}

// UDP通信单例类（**所有功能完整保留**）
class DataComm
{
private:
    // 私有构造/析构（单例模式）
    DataComm()
    {
        socket_handle = socket(AF_INET, SOCK_DGRAM, 0);
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        buffer = new char[2048];
        sz = 2048;
        service = 0;
    }

    ~DataComm()
    {
        close(socket_handle);
        stopReceiveService();
        delete[] buffer;
    }

    // 禁用拷贝和赋值（单例安全）
    DataComm(const DataComm &) = delete;
    DataComm &operator=(const DataComm &) = delete;

public:
    // 获取单例实例
    static DataComm *getInstance()
    {
        static DataComm comm;
        return &comm;
    }

    // 设置目标IP和端口
    void setDestAddress(const char *ip, int port)
    {
        addr.sin_port = htons((short)port);
        addr.sin_addr.s_addr = inet_addr(ip);
    }

    // 发送带换行的字符串
    int sendLine(const char *msg)
    {
        int n = strlen(msg);
        setBufferSize(n + 1);
        memcpy(buffer, msg, n);
        buffer[n] = '\n';
        return sendto(socket_handle, buffer, n + 1, 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    // 发送纯字符串
    int sendMsg(const char *msg)
    {
        int n = strlen(msg);
        return sendto(socket_handle, msg, n, 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    // 发送机器人状态数据（控制器核心使用）
    int sendRobotStatus(const RobotData &s)
    {
        return sendto(socket_handle, &s, sizeof(s), 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    // 启动接收服务（线程+回调+定时）
    void startReceiveService(ReceiveCallBack fnc, void *pClientData, int port, double dt = 0.0)
    {
        static ClientData data;
        if (service)
        {
            printf("there is already a receive thread\n");
            return;
        }
        data.fnc = fnc;
        data.port = port;
        data.dt = dt;
        data.pClientData = pClientData;
        int ret = pthread_create(&service, nullptr, service_func, &data);
        if (ret)
        {
            printf("create data communication thread failed\n");
        }
    }

    // 停止接收服务
    void stopReceiveService()
    {
        if (service)
        {
            loop = false;
            pthread_join(service, nullptr);
            service = 0;
        }
    }

private:
    // 动态调整发送缓冲区
    void setBufferSize(int n)
    {
        if (n > sz)
        {
            delete[] buffer;
            buffer = new char[n];
            sz = 2 * n;
        }
    }

private:
    int socket_handle;
    struct sockaddr_in addr;
    char *buffer;
    int sz;
    pthread_t service;
};

// 模板工具函数（控制器必用：填充RobotData数据）
template <class T>
inline void log2Channel(RobotData &roboData, int c, const T *data, int n, int offset = 0)
{
    for (int i = 0; i < n; i++)
        roboData.q[c][offset + i] = static_cast<float>(data[i]);
}

#endif // ROS2_UTILITY_DATA_COMM_HPP