#ifndef SESSION5_TURN_INTEGRATED_H_
#define SESSION5_TURN_INTEGRATED_H_

/**
 * Session 5: 转向系统（整合版）
 * 功能：
 * 1. 根据指示牌方向执行定时转向 + 定时回正
 * 2. 转向完成后继续巡线
 * 3. 检测黄色锥桶(y_obs)状态
 * 4. 若检测到y_obs后又消失，等待0.5s后重复转向流程
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <deque>
#include <mutex>
#include <queue>
#include "ros_serial/to32.h"
#include "obstacle_detector/PerceptionTargets.h"

class Session5TurnIntegrated {
public:
    Session5TurnIntegrated();
    ~Session5TurnIntegrated() = default;

    // 设置转向方向 (1: left, -1: right)
    void SetDirection(int direction);
    
    // 执行完整转向流程（阻塞式）
    void Execute();
    
    // 检查是否完成
    bool IsComplete() const;
    
    // 获取y_obs检测标志
    int GetYObsFlag() const { return y_obs_flag_; }

private:
    ros::NodeHandle nh_;
    ros::Subscriber targets_subscriber_;
    ros::Subscriber direction_subscriber_;
    ros::Subscriber point_subscriber_;
    ros::Publisher publisher_;
    
    // 参数
    float lineSpeed_ = 1.8;
    float follow_p_ = 0.1;
    float turn_angle_left_ = 10.0;   // 左转角度
    float turn_angle_right_ = 8.0;   // 右转角度
    float turn_duration_left_ = 7.0;      // 左转时间（秒）
    float turn_duration_right_ = 5.7;      // 右转时间（秒）
    float correct_duration_left_ = 6.0;   // 左回正时间（秒）
    float correct_duration_right_ = 4.0;   // 右回正时间（秒）
    float y_obs_wait_timeout_ = 1.0; // y_obs消失后等待时间（秒）
    
    // 状态
    int direction_ = 1;              // 转向方向 (1: left, -1: right)
    int y_obs_flag_ = 0;             // y_obs检测标志 (0: 未检测, 1: 已检测)
    bool y_obs_detected_ = false;    // 当前帧是否检测到y_obs
    int y_obs_detect_count_ = 0;     // y_obs 连续检测到的帧数
    int y_obs_detect_threshold_ = 3; // 连续检测多少帧才确认为检测到
    bool complete_ = false;

    // 巡线/避障状态
    std::mutex mutex_;
    std::priority_queue<obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector<obstacle_detector::PerceptionTargets::ConstPtr>> point_queue_;
    float adjusted_center_ = 355.0;
    std::deque<float> error_buffer_;
    std::deque<float> angle_buffer_;
    size_t filter_window_size_ = 3;
    float last_angle_ = 0.0;
    float max_angle_diff_ = 5.0;
    bool first_frame_ = true;
    
    // 回调函数
    void TargetsCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    void DirectionCallback(const std_msgs::String::ConstPtr& msg);
    void PointCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    
    // 辅助函数
    void PublishCommand(float speed, float angle);
    void ExecuteTurnAndCorrect();    // 执行一次完整的转向+回正
    bool WaitForYObsDisappear();     // 等待y_obs消失，返回是否需要重复转向
    void ProcessLineAndObstacle();   // 巡线+避障一次循环
    void LineFollowing(const obstacle_detector::Target& point_msg, float add_speed = 0.0);
    float ApplyMovingAverage(std::deque<float>& buffer, float new_value);
    bool IsAngleValid(float current_angle);
};

#endif  // SESSION5_TURN_INTEGRATED_H_
