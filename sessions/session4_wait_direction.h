#ifndef SESSION4_WAIT_DIRECTION_H_
#define SESSION4_WAIT_DIRECTION_H_

/**
 * Session 4: 斑马线前停车，等待方向指示
 * 流程：
 * 1. 巡线前进
 * 2. 检测到 zebra (zebra_detected_ = 1)
 * 3. zebra 消失 (zebra_detected_ = 2)，说明已经靠近
 * 4. 停车等待 10s，同时检测方向指示
 */

#include <atomic>
#include <mutex>
#include <queue>
#include <deque>
#include <unordered_map>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "obstacle_detector/PerceptionTargets.h"
#include "obstacle_detector/PerceptionTarget.h"
#include "obstacle_detector/Target.h"
#include "ros_serial/to32.h"

class Session4WaitDirection {
public:
    Session4WaitDirection();
    ~Session4WaitDirection() = default;

    // 执行主循环
    void Execute();
    
    // 检查是否收到方向消息
    bool HasDirection() const;
    
    // 获取方向 (1: left, -1: right, 0: none)
    int GetDirection() const;
    
    // 检查是否超时
    bool IsTimeout() const;
    
    // 重置状态
    void Reset();
    
    // 检查是否已完成（停车超时或收到方向）
    bool IsComplete() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber direction_subscriber_;
    ros::Subscriber zebra_subscriber_;
    ros::Subscriber points_subscriber_;
    ros::Publisher publisher_;
    
    ros::Time start_time_;           // 停车开始时间
    ros::Time last_zebra_time_;      // 上次检测到 zebra 的时间
    std::atomic<int> direction_{0};  // 方向: 1=left, -1=right, 0=none
    
    float wait_timeout_ = 11.0;      // 最大等待时间
    float default_direction_ = 1.0;  // 默认方向 (1: left)
    float zebra_disappear_timeout_ = 0.5; // zebra 消失判定时间(秒)
    
    bool started_ = false;           // 停车计时是否开始
    std::atomic<int> zebra_detected_{0}; // 0=未检测, 1=检测到, 2=消失(可以停车)
    
    // 巡线相关
    std::mutex mutex_;
    std::priority_queue<obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector<obstacle_detector::PerceptionTargets::ConstPtr>> point_queue_;
    std::unordered_map<uint64_t, float> left_x_, right_x_;
    
    // 巡线参数
    float lineSpeed_ = 1.8;
    float follow_p_ = 0.10;
    std::queue<float> correctAngle_;
    bool iscorrecting_ = false;
    
    // 滤波参数
    std::deque<float> error_buffer_;
    std::deque<float> angle_buffer_;
    size_t filter_window_size_ = 3;
    float last_angle_ = 0.0;
    float max_angle_diff_ = 5.0;
    bool first_frame_ = true;

    // 储存10个内的方向指示
    int direction_history_[10] = {0};  // 1=left, -1=right, 0=未收集
    int direction_count_ = 0;           // 已收集的方向数量
    bool collecting_direction_ = false; // 是否开始收集方向

    // 回调函数
    void DirectionCallback(const std_msgs::String::ConstPtr& msg);
    void ZebraCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    void PointsCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    
    // 巡线函数
    void LineFollowing(const obstacle_detector::Target& point_msg, float add_speed = 0.0);
    void PublishStop();
    
    // 辅助函数
    float ApplyMovingAverage(std::deque<float>& buffer, float new_value);
    bool IsAngleValid(float current_angle);
    void CheckZebraDisappear();  // 检查 zebra 是否消失
    int JudgeDirection() const;  // 根据收集的10个样本判断方向
};

#endif  // SESSION4_WAIT_DIRECTION_H_
