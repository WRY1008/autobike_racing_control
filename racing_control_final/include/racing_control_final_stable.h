#ifndef RACING_CONTROL_FINAL_H_
#define RACING_CONTROL_FINAL_H_

/**
 * Racing Control Final: 整合版控制节点
 * 
 * 状态机：
 * SESSION_2: 巡线 + 避障 + 减速带
 *   → 跳变条件: 连续检测到3帧斑马线
 * SESSION_4: 巡线 → 斑马线消失 → 停车11秒 → 收集方向
 *   → 跳变条件: 停车超过11秒 且 已收到方向
 * SESSION_5: 定时转向 → 回正 → 巡线 → 检测y_obs → 可能重复转向
 *   → 跳变条件: 完成所有转向流程
 * SESSION_COMPLETE: 完成
 */

#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <deque>
#include <unordered_map>
#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "obstacle_detector/PerceptionTargets.h"
#include "obstacle_detector/PerceptionTarget.h"
#include "obstacle_detector/Target.h"
#include "ros_serial/to32.h"

// 状态枚举
enum class SessionState {
    SESSION_2,          // 巡线 + 避障 + 减速带
    SESSION_4,          // 斑马线停车 + 等待方向
    SESSION_5,          // 转向 + y_obs检测
    SESSION_COMPLETE    // 完成
};

class RacingControlFinal {
public:
    RacingControlFinal();
    ~RacingControlFinal();

    // 主执行函数
    void Execute();

    // 停止
    void Stop();

    // 获取当前状态
    SessionState GetState() const { return current_state_; }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_subscriber_;
    ros::Subscriber target_subscriber_;
    ros::Subscriber direction_subscriber_;
    ros::Publisher publisher_;

    std::shared_ptr<std::thread> process_thread_;
    bool process_stop_ = false;
    bool sub_target_ = false;

    std::mutex mutex_;
    std::priority_queue<obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector<obstacle_detector::PerceptionTargets::ConstPtr>> point_queue_;
    std::priority_queue<obstacle_detector::PerceptionTargets::ConstPtr,
                        std::vector<obstacle_detector::PerceptionTargets::ConstPtr>> targets_queue_;

    // ==================== 状态机 ====================
    SessionState current_state_ = SessionState::SESSION_2;

    // ==================== 回调函数 ====================
    void PointCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    void TargetCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg);
    void DirectionCallback(const std_msgs::String::ConstPtr& msg);

    // ==================== Session执行函数 ====================
    void ExecuteSession2();
    void ExecuteSession4();
    void ExecuteSession5();

    // ==================== 共用参数 ====================
    float lineSpeed_ = 1.8;
    float follow_p_ = 0.1;
    int avoidBottom_ = 240;
    float default_direction_ = 0.0;  // 默认方向 (1: left)

    // ==================== 共用巡线状态 ====================
    float adjusted_center_ = 320.0;
    std::deque<float> error_buffer_;
    std::deque<float> angle_buffer_;
    size_t filter_window_size_ = 3;
    float last_angle_ = 0.0;
    float max_angle_diff_ = 5.0;
    bool first_frame_ = true;
    std::unordered_map<uint64_t, float> left_x_, right_x_;

    // ==================== 赛道恢复相关 ====================
    int abnormal_lane_count_ = 0;        // 连续异常帧计数
    bool is_recovering_ = false;          // 是否正在执行恢复动作
    ros::Time recovery_start_time_;       // 恢复动作开始时间
    float recovery_angle_ = 0.0f;         // 恢复时使用的角度
    
    // 参数
    int abnormal_threshold_ = 10;         // 触发恢复的阈值（默认10帧）
    float recovery_duration_ = 2.0f;      // 恢复持续时间（默认2秒）

    // ==================== Session 2: 避障相关 ====================
    float avoid_p_ = 0.25;
    bool judgeLeft_ = false;
    bool isAvoiding_ = false;
    bool isSlowing_ = false;
    std::string avoidingClass_ = "";
    int noObstacleCount_ = 0;
    int noObstacleThreshold_ = 20;
    float time_extend_ = 1.0; // 减速带延长加速时间
    int slow_flag_ = 0;
    ros::Time slow_last_time_;

    // Session 2 → Session 4: 斑马线检测
    int zebra_detect_count_ = 0;
    int zebra_detect_threshold_ = 5;  // 连续5帧检测到斑马线
    int zebra_valid_bottom_y_ = 320;  // 斑马线框底边 y2 达到该阈值才算“有效检测”

    // ==================== Session 4: 斑马线停车相关 ====================
    std::atomic<int> zebra_detected_{0};  // 0=未检测, 1=检测到, 2=消失
    ros::Time last_zebra_time_;
    float zebra_disappear_timeout_ = 1.0f; //未检测到后多长时间认定为消失，及时停下

    ros::Time stop_start_time_;
    bool stop_started_ = false;
    float wait_timeout_ = 11.0;

    // 方向收集
    float zebra_direction_start_bottom = 400.0f; // zebra 检测框底边 y2 达到该阈值后开始收集方向
    std::atomic<int> direction_{0};
    int direction_history_[10] = {0};
    int direction_count_ = 0;
    bool collecting_direction_ = false;

    // Session 4: 指示牌检测框中心 (用于动态计算转向角/时间)
    float sign_center_sum_x_ = 0.0f;
    int sign_center_count_ = 0;
    float latest_sign_center_x_ = 350.0f;
    bool latest_sign_center_valid_ = false;

    // 动态转向参数（仅用于“指示牌决定的第一次转向”）
    float turn_angle_from_sign_ = 0.0f;
    float turn_duration_from_sign_ = 0.0f;
    float turn_scale_from_sign_ = 1.0f;
    bool use_dynamic_turn_from_sign_ = false;

    // 是否启用“根据指示牌中心动态调整转向角/时间”（可用 ROS 参数控制）
    bool enable_dynamic_turn_from_sign_ = true;

    // 系数：根据指示牌中心偏移缩放角度/时间（0.0~1.0 推荐）
    float turn_center_coeff_ = 1.20f;

    // ==================== Session 5: 转向相关 ====================
    // 第一次变道参数
    float turn_angle_left_ = 12.0;
    float turn_angle_right_ = 12.0;
    float turn_duration_left_ = 4.8;
    float turn_duration_right_ = 6.2;
    float correct_duration_left_ = 4.4;
    float correct_duration_right_ = 5.2;
    
    // 第二次变道参数（超车/回到车道）
    float turn2_angle_left_ = 14.0;
    float turn2_angle_right_ = 12.0;
    float turn2_duration_left_ = 10.5;
    float turn2_duration_right_ = 11.5;
    float correct2_duration_left_ = 8.5;
    float correct2_duration_right_ = 12.5;

    // y_obs 消失后等待确认的“帧数”（连续未检测到多少帧才认为确认消失）
    int y_obs_wait_timeout_ = 10;
    std::atomic<uint64_t> session5_last_target_seq_{0};

    int y_obs_flag_ = 0;
    bool y_obs_detected_ = false;
    int y_obs_detect_count_ = 0;
    int y_obs_detect_threshold_ = 3;
    bool session5_complete_ = false;
    int turn_count_ = 0;
    int original_direction_ = 1;

    // 看到黄桶所产生的巡线偏移量，默认40，记得改！
    float y_obs_offset_ = 30.0f;

    // Session 5完成后的冲刺速度增量（在 lineSpeed_ 基础上增加）
    float final_sprint_speed_ = 0.5f;

    // ==================== 辅助函数 ====================
    void LineFollowing(const obstacle_detector::Target& point_msg, float add_speed = 0.0);
    void UpdateAdjustedCenter(const obstacle_detector::PerceptionTarget& target);
    void PublishCommand(float speed, float angle);
    void PublishStop();
    float ApplyMovingAverage(std::deque<float>& buffer, float new_value);
    bool IsAngleValid(float current_angle);

    // Session 4 辅助
    void CheckZebraDisappear();
    int JudgeDirection() const;

    // 根据指示牌检测框中心 x 计算转向角/时间（满足 x≈320 使用原值）
    void ComputeTurnFromSignCenter(int dir, float sign_center_x, float& out_angle, float& out_duration, float& out_scale) const;

    // Session 5 辅助
    void ExecuteTurnAndCorrect();
    bool WaitForYObsDisappear();
    void ProcessLineAndObstacle(float add_speed = 0.0f);
};

#endif  // RACING_CONTROL_FINAL_H_
