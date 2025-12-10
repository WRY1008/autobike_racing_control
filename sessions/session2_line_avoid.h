#ifndef SESSION2_LINE_AVOID_H_
#define SESSION2_LINE_AVOID_H_

/**
 * Session 2: 巡线 + 避障
 * Mode 2 - 跟随车道线行驶，遇到障碍物时通过调整目标中心点来避障
 */

#include <thread>
#include <mutex>
#include <queue>
#include <deque>
#include <unordered_map>

#include <ros/ros.h>
#include "obstacle_detector/PerceptionTargets.h"
#include "obstacle_detector/PerceptionTarget.h"
#include "ros_serial/to32.h"

class Session2LineAvoid {
public:
    Session2LineAvoid();
    ~Session2LineAvoid();

    // 主处理循环
    void Execute();

    // 停止
    void Stop();

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_subscriber_;
    ros::Subscriber target_subscriber_;
    ros::Publisher publisher_;

    std::shared_ptr<std::thread> process_thread_;
    bool process_stop_ = false;
    bool sub_target_ = false;

    std::mutex mutex_;
    std::priority_queue<obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector<obstacle_detector::PerceptionTargets::ConstPtr>> point_queue_;
    std::priority_queue<obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector<obstacle_detector::PerceptionTargets::ConstPtr>> targets_queue_;

    // 回调函数
    void PointCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    void TargetCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);

    // 核心功能
    void LineFollowing(const obstacle_detector::Target& point_msg, float add_speed = 0.0);
    void UpdateAdjustedCenter(const obstacle_detector::PerceptionTarget& target);

    // 参数
    int avoidBottom_ = 240;           // 障碍物触发避障的下边界
    float lineSpeed_ = 1.8;           // 巡线速度
    float follow_p_ = 0.1;            // 巡线PID参数
    float avoid_p_ = 0.25;            // 避障PID参数（已废弃，保留以兼容参数读取）
    bool judgeLeft_ = false;          // 避障方向标志
    bool iscorrecting_ = false;       // 是否在回正中
    bool isAvoiding_ = false;         // 是否在避障中
    std::string avoidingClass_ = ""; // 正在避让的障碍物类型
    int noObstacleCount_ = 0;         // 连续未检测到障碍物的帧数
    int noObstacleThreshold_ = 5;     // 连续多少帧未检测到才认为障碍物消失
    float adjusted_center_ = 350.0;   // 动态调整的目标中心点
    std::queue<float> correctAngle_;  // 回正角度队列
    std::unordered_map<int, float> left_x_, right_x_;

    // 滤波器参数
    std::deque<float> error_buffer_;      // error 缓冲区
    std::deque<float> angle_buffer_;      // angle 缓冲区
    size_t filter_window_size_ = 3;       // 滤波窗口大小
    float last_angle_ = 0.0;              // 上一帧的角度
    float max_angle_diff_ = 5.0;          // 最大允许角度变化
    bool first_frame_ = true;             // 是否是第一帧

    // 辅助函数
    float ApplyMovingAverage(std::deque<float>& buffer, float new_value);
    bool IsAngleValid(float current_angle);
};

#endif  // SESSION2_LINE_AVOID_H_
