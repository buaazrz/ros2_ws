#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>

namespace robot_math
{
    template <class T>
    class KalmanFilter
    {
    public:
        // Q: 过程噪声 (信任模型的程度，越小越平滑，建议 0.01~0.001)
        // R: 测量噪声 (信任传感器的程度，越大越平滑，建议 0.1~1.0)
        KalmanFilter(int nChannel, T Q = 0.01, T R = 0.1);
        ~KalmanFilter();
        
        void filtering(const T *datain, T *dataout);
        void reset();

    private:
        int Channel;
        T Q_noise; 
        T R_noise;
        bool is_initialized;

        std::vector<T> x; // 状态估计值 (如: 滤波后的速度)
        std::vector<T> P; // 估计协方差 (如: 误差的不确定度)
    };

    template <class T>
    KalmanFilter<T>::KalmanFilter(int nChannel, T Q, T R) : x(nChannel, 0), P(nChannel, 1.0)
    {
        Channel = nChannel;
        Q_noise = Q;
        R_noise = R;
        is_initialized = false;
    }

    template <class T>
    KalmanFilter<T>::~KalmanFilter()
    {
    }

    template <class T>
    void KalmanFilter<T>::reset()
    {
        for (int i = 0; i < Channel; i++)
        {
            x[i] = 0;
            P[i] = 1.0;
        }
        is_initialized = false;
    }

    template <class T>
    void KalmanFilter<T>::filtering(const T *datain, T *dataout)
    {
        // 如果是第一次滤波，直接用测量值初始化状态，防止启动突变
        if (!is_initialized)
        {
            for (int i = 0; i < Channel; i++)
            {
                x[i] = datain[i];
                P[i] = 1.0;
                if (dataout != nullptr) {
                    dataout[i] = x[i];
                }
            }
            is_initialized = true;
            return;
        }

        for (int i = 0; i < Channel; i++)
        {
            // 1. 预测 (Predict)
            // 假设速度不变模型，所以预测的 x 不变，仅增加不确定度 P
            P[i] = P[i] + Q_noise;

            // 2. 更新 (Update)
            // 计算卡尔曼增益 K
            T K = P[i] / (P[i] + R_noise); 
            // 修正状态估计 x
            x[i] = x[i] + K * (datain[i] - x[i]);
            // 更新协方差 P
            P[i] = (1 - K) * P[i];

            // 3. 输出
            if (dataout != nullptr) {
                dataout[i] = x[i];
            }
        }
    }
}
#endif // KALMANFILTER_H