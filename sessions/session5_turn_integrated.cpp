#include "session5_turn.h"
#include <algorithm>
#include <cmath>

/**
 * Session 5: 转向系统（整合版）
 * 
 * 执行流程：
 * ┌─────────────────────────────────────────────────────────────┐
 * │ 1. 接收方向指令，设置转向方向                                 │
 * └─────────────────────────────────────────────────────────────┘
 *                           ↓
 * ┌─────────────────────────────────────────────────────────────┐
 * │ 2. 执行定时转向 (turn_duration_)                             │
 * │    - 发布转向角度指令                                        │
 * │    - 同时监测y_obs                                           │
 * └─────────────────────────────────────────────────────────────┘
 *                           ↓
 * ┌─────────────────────────────────────────────────────────────┐
 * │ 3. 执行定时回正 (correct_duration_)                         │
 * │    - 发布回正角度指令                                       │
 * │    - 继续监测y_obs                                          │
 * └─────────────────────────────────────────────────────────────┘
 *                           ↓
 * ┌─────────────────────────────────────────────────────────────┐
 * │ 4. 检查y_obs状态                                            │
 * │    ├─ y_obs_flag_ = 0 (从未检测到)                         │
 * │    │   → 直接结束，继续巡线                                 │
 * │    │                                                        │
 * │    └─ y_obs_flag_ = 1 (检测到过)                           │
 * │        └─ 当前y_obs消失                                     │
 * │            → 等待 0.5s (y_obs_wait_timeout_)               │
 * │            → 重复步骤2-4                                    │
 * └─────────────────────────────────────────────────────────────┘
 */

Session5TurnIntegrated::Session5TurnIntegrated() {
    point_subscriber_ = nh_.subscribe("/line_detector/center", 10,
                                       &Session5TurnIntegrated::PointCallback, this);
    targets_subscriber_ = nh_.subscribe("/obstacle/detection", 10, 
                                        &Session5TurnIntegrated::TargetsCallback, this);
    direction_subscriber_ = nh_.subscribe("/yolo_fastestv2/direction", 10, 
                                        &Session5TurnIntegrated::DirectionCallback, this);
    publisher_ = nh_.advertise<ros_serial::to32>("/cmd_vel", 5);
    
    nh_.getParam("lineSpeed", lineSpeed_);
    nh_.getParam("follow_p", follow_p_);
    nh_.getParam("turn_angle_left", turn_angle_left_);
    nh_.getParam("turn_angle_right", turn_angle_right_);
    int filter_window_tmp = static_cast<int>(filter_window_size_);
    nh_.getParam("filter_window_size", filter_window_tmp);
    filter_window_size_ = static_cast<size_t>(filter_window_tmp);
    nh_.getParam("max_angle_diff", max_angle_diff_);
    nh_.getParam("turn_duration_left", turn_duration_left_);
    nh_.getParam("turn_duration_right", turn_duration_right_);
    nh_.getParam("correct_duration_left", correct_duration_left_);
    nh_.getParam("correct_duration_right", correct_duration_right_);
    nh_.getParam("y_obs_wait_timeout", y_obs_wait_timeout_);
    
    ROS_INFO("Session5TurnIntegrated initialized!");
    ROS_INFO("Parameters: lineSpeed=%.2f, turn_angle_left=%.1f, turn_angle_right=%.1f, turn_duration_left=%.1fs, turn_duration_right=%.1fs, correct_duration_left=%.1fs, correct_duration_right=%.1fs, wait_timeout=%.1fs", 
             lineSpeed_, turn_angle_left_, turn_angle_right_, turn_duration_left_, turn_duration_right_, correct_duration_left_, correct_duration_right_, y_obs_wait_timeout_);
}

void Session5TurnIntegrated::DirectionCallback(const std_msgs::String::ConstPtr& msg){
    if(msg->data == "left"){
        direction_ = 1;
        ROS_INFO("Direction received: LEFT - Starting turn execution...");
        Execute();  // 自动执行转向
    } else if(msg->data == "right"){
        direction_ = -1;
        ROS_INFO("Direction received: RIGHT - Starting turn execution...");
        Execute();  // 自动执行转向
    }
     else {
        ROS_WARN("Unknown direction: %s", msg->data.c_str());
    }
}

void Session5TurnIntegrated::PointCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
    std::unique_lock<std::mutex> lock(mutex_);
    point_queue_.push(msg);
    if (point_queue_.size() > 1) {
        point_queue_.pop();
    }
}

void Session5TurnIntegrated::TargetsCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
    // 检测y_obs
    bool current_frame_detected = false;
    
    for (const auto& obstacle : msg->obstacles) {
        if (obstacle.obj_class == "y_obs") {
            current_frame_detected = true;
            break;
        }
    }
    
    y_obs_detected_ = current_frame_detected;
    
    // 连续检测逻辑：从 0 → 1 需要连续 3 帧
    if (y_obs_flag_ == 0 && current_frame_detected) {
        y_obs_detect_count_++;
        ROS_INFO("Y_OBS detected, count: %d/%d", y_obs_detect_count_, y_obs_detect_threshold_);
        
        if (y_obs_detect_count_ >= y_obs_detect_threshold_) {
            y_obs_flag_ = 1;
            ROS_INFO("Y_OBS CONFIRMED! Flag set to 1 after %d consecutive frames", y_obs_detect_count_);
            y_obs_detect_count_ = 0;  // 重置计数器
        }
    } else if (!current_frame_detected) {
        // 未检测到，重置计数器
        if (y_obs_detect_count_ > 0) {
            ROS_INFO("Y_OBS detection interrupted, resetting count");
        }
        y_obs_detect_count_ = 0;
    }
}

void Session5TurnIntegrated::SetDirection(int direction) {
    direction_ = (direction > 0) ? 1 : -1;
    complete_ = false;
    y_obs_flag_ = 0;  // 重置y_obs标志
    y_obs_detected_ = false;
    y_obs_detect_count_ = 0;  // 重置计数器
    
    ROS_INFO("Turn direction set to: %s", direction_ > 0 ? "LEFT" : "RIGHT");
}

void Session5TurnIntegrated::PublishCommand(float speed, float angle) {
    ros_serial::to32 msg;
    msg.speed = speed;
    msg.angle = angle;
    msg.run_flag = true;
    publisher_.publish(msg);
}

void Session5TurnIntegrated::ExecuteTurnAndCorrect() {
    float isLeft = (direction_ > 0) ? 1.0f : -1.0f;
    float turn_angle = (direction_ > 0) ? turn_angle_left_ : turn_angle_right_;
    float turn_duration = (direction_ > 0) ? turn_duration_left_ : turn_duration_right_;
    float correct_duration = (direction_ > 0) ? correct_duration_left_ : correct_duration_right_;
    
    // ========== 阶段1: 转向 ==========
    ROS_INFO("Phase 1: Turning %s for %.1f seconds...", 
             direction_ > 0 ? "LEFT" : "RIGHT", turn_duration);
    ros::Time start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < turn_duration) {
        // 转向时不处理回调，不更新 y_obs 状态
        
        float angle = isLeft * turn_angle;
        ROS_INFO_THROTTLE(1, "Turning: angle=%.1f", angle);
        PublishCommand(lineSpeed_, angle);
        
        ros::Duration(0.1).sleep();
    }
    
    // ========== 阶段2: 回正 ==========
    ROS_INFO("Phase 2: Correcting for %.1f seconds...", correct_duration);
    start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < correct_duration) {
        // 回正时也不处理回调
        
        float angle = -0.8f * isLeft * turn_angle;
        ROS_INFO_THROTTLE(1, "Correcting: angle=%.1f", angle);
        PublishCommand(lineSpeed_, angle);
        
        ros::Duration(0.1).sleep();
    }
    
    ROS_INFO("Turn and correct cycle completed! Now starting line following and y_obs detection...");
}

bool Session5TurnIntegrated::WaitForYObsDisappear() {
    ROS_INFO("Starting y_obs detection phase, current y_obs_flag=%d", y_obs_flag_);
    
    ros::Time wait_start;  // 记录 y_obs 消失的时间
    bool disappear_timer_started = false;  // 是否已开始计时
    
    // 持续检测循环：等待 y_obs 从 0→1→2 的状态转换
    while (ros::ok()) {
        ros::spinOnce();  // 处理回调更新 y_obs_detected_
        ProcessLineAndObstacle();  // 巡线
        
        // 检查当前帧是否检测到 y_obs
        if (y_obs_detected_) {
            ROS_INFO_THROTTLE(0.2, "Y_OBS present in frame...");
        }
        
        // 状态转换逻辑
        if (y_obs_flag_ == 1) {
            // 已经确认检测到 y_obs（连续3帧）
            if (!y_obs_detected_) {
                // y_obs 消失了，转换到状态 2，开始计时
                if (!disappear_timer_started) {
                    ROS_INFO("Y_OBS disappeared! Changing flag from 1 to 2, starting %.1fs wait...", y_obs_wait_timeout_);
                    y_obs_flag_ = 2;
                    wait_start = ros::Time::now();  // 开始计时
                    disappear_timer_started = true;
                }
            }
        } else if (y_obs_flag_ == 2) {
            // y_obs 已消失，等待确认
            double elapsed = (ros::Time::now() - wait_start).toSec();
            
            if (y_obs_detected_) {
                // 又检测到了，回到状态 1，重置计时器
                ROS_INFO("Y_OBS reappeared! Changing flag back from 2 to 1");
                y_obs_flag_ = 1;
                disappear_timer_started = false;
            } else if (elapsed >= y_obs_wait_timeout_) {
                // 确认消失超过 0.5s，触发重复转弯
                ROS_WARN("Y_OBS confirmed disappeared for %.1fs, repeating turn maneuver...", y_obs_wait_timeout_);
                complete_ = true;
                return true;  // 需要重复转弯
            } else {
                ROS_INFO_THROTTLE(0.2, "Waiting for y_obs disappear confirmation: %.2fs / %.1fs", 
                                 elapsed, y_obs_wait_timeout_);
            }
        } else if (y_obs_flag_ == 0) {
            // 等待检测到 y_obs（从 0→1 需要连续3帧，在 TargetsCallback 里处理）
            ROS_INFO_THROTTLE(1.0, "Waiting for y_obs detection (flag=0)...");
        }
        
        ros::Duration(0.05).sleep();
    }
    
    // 正常情况不会到这里（只有 ros::ok() 为 false 才会）
    return false;
}

void Session5TurnIntegrated::Execute() {
    ROS_INFO("=== Starting Session 5 Turn Integrated ===");
    
    int original_direction = direction_;  // 保存原始方向
    int turn_count = 0;  // 转弯次数计数
    
    // 执行转向流程，可能多次重复
    while (ros::ok()) {
        turn_count++;
        
        // 第一次：按原始方向转弯避障
        // 第二次：反向转弯回到原车道（超车逻辑）
        if (turn_count == 1) {
            ROS_INFO("=== Turn #1: Avoiding obstacle, direction=%s ===", 
                     direction_ > 0 ? "LEFT" : "RIGHT");
        } else if (turn_count == 2) {
            direction_ = -original_direction;  // 反向
            ROS_INFO("=== Turn #2: Returning to lane (overtaking), direction=%s ===", 
                     direction_ > 0 ? "LEFT" : "RIGHT");
        }
        
        // 执行一次完整的转向+回正（不处理回调）
        ExecuteTurnAndCorrect();
        
        // 回正结束后，开始巡线并检测 y_obs
        ROS_INFO("Turn/Correct finished, now processing line following and y_obs detection...");

        // 第二次转向完成后：直接进入持续巡线（必须 spinOnce 才会有新点进队列）
        if (turn_count >= 2) {
            ros::Rate rate(30);
            while (ros::ok()) {
                ROS_INFO("=== Turn #2: Following line ===");
                ros::spinOnce();
                ProcessLineAndObstacle();
                rate.sleep();
            }
            break;
        }
        
        // 检查是否需要重复
        bool need_repeat = false;
        if (turn_count < 2) {
            need_repeat = WaitForYObsDisappear();
        }

        if (!need_repeat) {
            break;  // 不需要重复，退出循环
        }

        // 需要重复，继续下一轮循环
        ROS_INFO("Y_OBS disappeared, repeating turn maneuver...");
    }
    
    complete_ = true;
    ROS_INFO("Session 5 Turn Integrated complete! Final y_obs_flag=%d, total turns=%d", 
             y_obs_flag_, turn_count);
    ROS_INFO("Returning to line following mode...");
}

void Session5TurnIntegrated::ProcessLineAndObstacle() {
    std::unique_lock<std::mutex> lock(mutex_);

    // 仅巡线：从最新点云执行一次巡线控制
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        if (!point_msg->lane_points.empty()) {
            LineFollowing(point_msg->lane_points[0], 0.0f);
        }
        lock.lock();
        point_queue_.pop();
    }
}

void Session5TurnIntegrated::LineFollowing(const obstacle_detector::Target& point_msg, float add_speed) {
    ros_serial::to32 to32_msg;
    float angle = 0.0f;

    ROS_INFO("LineFollowing...");

    if (point_msg.points.empty() || point_msg.points[0].point.size() < 3) {
        ROS_WARN("Invalid point data, skipping frame");
        return;
    }

    float left = point_msg.points[0].point[1].x;
    float right = point_msg.points[0].point[2].x;
    float mid = point_msg.points[0].point[0].x;

    ROS_INFO("left_x:%.1f, mid_x:%.1f, right_x:%.1f", left, mid, right);

    if (right - left < 100 || right - left > 200) {
        ROS_WARN("Lane width abnormal (%.1f), using last valid angle", right - left);
        angle = last_angle_ * 0.8f;
        angle = std::max(-10.0f, std::min(10.0f, angle));
        
        to32_msg.speed = lineSpeed_ * 0.7f;
        to32_msg.angle = angle;
        to32_msg.run_flag = true;
        publisher_.publish(to32_msg);
        return;
    }

    float error = mid - adjusted_center_;
    ROS_INFO("NORMAL: mid=%.1f, error=%.1f", mid, error);

    float filtered_error = ApplyMovingAverage(error_buffer_, error);
    ROS_INFO("error: raw=%.1f, filtered=%.1f", error, filtered_error);

    angle = -1.0f * follow_p_ * filtered_error;
    angle = std::max(-10.0f, std::min(10.0f, angle));

    if (!IsAngleValid(angle)) {
        ROS_WARN("Angle change too large (current=%.1f, last=%.1f), skipping frame", angle, last_angle_);
        return;
    }

    angle = ApplyMovingAverage(angle_buffer_, angle);
    last_angle_ = angle;
    first_frame_ = false;

    ROS_INFO("target_angle: %.1f", angle);

    to32_msg.speed = lineSpeed_ + add_speed;
    to32_msg.angle = angle;
    to32_msg.run_flag = true;
    publisher_.publish(to32_msg);
}

float Session5TurnIntegrated::ApplyMovingAverage(std::deque<float>& buffer, float new_value) {
    buffer.push_back(new_value);

    if (buffer.size() > filter_window_size_) {
        buffer.pop_front();
    }

    float sum = 0.0f;
    for (float val : buffer) {
        sum += val;
    }

    return sum / buffer.size();
}

bool Session5TurnIntegrated::IsAngleValid(float current_angle) {
    if (first_frame_) {
        return true;
    }

    float diff = std::abs(current_angle - last_angle_);
    return diff <= max_angle_diff_;
}

bool Session5TurnIntegrated::IsComplete() const {
    return complete_;
}

// 测试用 main 函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "session5_turn_test");
    
    Session5TurnIntegrated session;
    
    ROS_INFO("=== Session 5 Integrated Test: Turn + Y_OBS Detection ===");
    ROS_INFO("Waiting for direction command from /yolo_fastestv2/direction topic...");
    ROS_INFO("Or you can call session.SetDirection(1) for LEFT or session.SetDirection(-1) for RIGHT");
    ROS_INFO("--------------------------------------------------------------");
    ROS_INFO("Logic:");
    ROS_INFO("1. Wait for direction from topic or manually set");
    ROS_INFO("2. Execute turn + correct");
    ROS_INFO("3. If y_obs never detected (flag=0): End and continue line following");
    ROS_INFO("4. If y_obs detected (flag=1) and then disappears:");
    ROS_INFO("   - Wait 0.5s");
    ROS_INFO("   - Repeat turn + correct");
    ROS_INFO("--------------------------------------------------------------");
    
    // 等待方向指令或手动触发（测试时可以取消注释）
    // session.SetDirection(1);  // 1=LEFT, -1=RIGHT
    
    ros::spin();  // 等待回调
    
    ROS_INFO("Session 5 complete! Final y_obs_flag: %d", session.GetYObsFlag());
    return 0;
}
