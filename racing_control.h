#ifndef RACING_CONTROL_H_
#define RACING_CONTROL_H_

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "obstacle_detector/PerceptionTargets.h"
#include "obstacle_detector/PerceptionTarget.h"
#include "ros_serial/to32.h"

// 根据 mode.md 定义的模式枚举
enum class RacingMode {
    STOP = 0,            // mode 0: 原地静止
    LINE_FOLLOW = 1,     // mode 1: 只参与巡线
    LINE_AVOID = 2,      // mode 2: 巡线 + 避障
    LINE_SPEED_BUMP = 3, // mode 3: 巡线 + 过减速带
    TURN = 4             // mode 4: 转向 + 回正（写死）
};


class RacingControlNode {
public:
    RacingControlNode(const std::string& node_name);
    ~RacingControlNode();
private:
    ros::Subscriber point_subscriber_, target_subscriber_, direction_subscriber_;
    ros::Publisher publisher_;
    std::shared_ptr<std::thread> msg_process_;
    bool process_stop_ = false, sub_target_ = false;
    std::mutex point_target_mutex_;
    std::priority_queue< obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector< obstacle_detector::PerceptionTargets::ConstPtr > > point_queue_;
    std::priority_queue< obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector< obstacle_detector::PerceptionTargets::ConstPtr > > targets_queue_;

    void subscription_callback_point(const obstacle_detector::PerceptionTargets::ConstPtr& point_msg);
    void subscription_callback_target(const obstacle_detector::PerceptionTargets::ConstPtr& targets_msg);
    void direction_callback(const std_msgs::String::ConstPtr& msg);
    void MessageProcess(void);
    void LineFollowing(const obstacle_detector::PerceptionTarget& point_msg, float add_speed = 0.0);
    void ObstaclesAvoiding(const obstacle_detector::PerceptionTarget& target, float add_speed = 0.0);
    void PublishStop();           // 发布停止指令
    void HandleModeStop();        // 处理 mode 0
    void HandleModeLineFollow();  // 处理 mode 1
    void HandleModeLineAvoid();   // 处理 mode 2
    void HandleModeSpeedBump();   // 处理 mode 3
    void HandleModeTurn();        // 处理 mode 4
    void BlessOurTurn();          // 转向执行
    void SwitchToNextSession();   // 切换到下一个 session

    int avoidBottom = 280;
    float LeftOrRight = 1.0;
    float lineSpeed = 1.8;
    std::atomic<int> latest_direction_{0};

    std::queue < float > correctAngle;
    bool iscorrecting = false;
    float follow_p = 0.06;

    // 模式状态机相关
    RacingMode current_mode_ = RacingMode::STOP;
    int current_session_ = 1;       // 当前 session (1-5)
    ros::Time race_start_time_;     // 比赛开始时间
    float initial_wait_time_ = 30.0; // session 1 静止等待时间
};

#endif  // RACING_CONTROL_H_
