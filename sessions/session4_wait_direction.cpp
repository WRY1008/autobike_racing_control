#include "session4_wait_direction.h"
#include <algorithm>
#include <cmath>

/**
 * Session 4: 斑马线前停车，等待方向指示
 * 流程：
 * 1. 巡线前进
 * 2. 检测到 zebra (zebra_detected_ = 1)
 * 3. zebra 消失 (zebra_detected_ = 2)，说明已经靠近
 * 4. 停车等待 10s，同时检测方向指示
 */

Session4WaitDirection::Session4WaitDirection() {
    direction_subscriber_ = nh_.subscribe("/yolo_fastestv2/direction", 1, 
                                           &Session4WaitDirection::DirectionCallback, this);
    zebra_subscriber_ = nh_.subscribe("/obstacle/detection", 1, 
                                       &Session4WaitDirection::ZebraCallback, this);
    points_subscriber_ = nh_.subscribe("/line_detector/center", 10, 
                                        &Session4WaitDirection::PointsCallback, this);
    publisher_ = nh_.advertise<ros_serial::to32>("/cmd_vel", 5);
    
    nh_.getParam("wait_timeout", wait_timeout_);
    nh_.getParam("LeftOrRight", default_direction_);
    nh_.getParam("lineSpeed", lineSpeed_);
    nh_.getParam("follow_p", follow_p_);
    nh_.getParam("zebra_disappear_timeout", zebra_disappear_timeout_);
    
    zebra_detected_.store(0);
    last_zebra_time_ = ros::Time::now();
    
    ROS_INFO("Session4WaitDirection initialized!");
    ROS_INFO("Parameters: wait_timeout=%.1fs, default_direction=%s, zebra_timeout=%.2fs", 
             wait_timeout_, default_direction_ > 0 ? "left" : "right", zebra_disappear_timeout_);
}

void Session4WaitDirection::Reset() {
    started_ = false;
    direction_.store(0);
    zebra_detected_.store(0);
    first_frame_ = true;
    error_buffer_.clear();
    angle_buffer_.clear();
    while (!correctAngle_.empty()) correctAngle_.pop();
    
    // 重置方向收集
    collecting_direction_ = false;
    direction_count_ = 0;
    for (int i = 0; i < 10; i++) {
        direction_history_[i] = 0;
    }
}

void Session4WaitDirection::DirectionCallback(const std_msgs::String::ConstPtr& msg) {
    // 只有在停车后才开始收集方向
    if (!collecting_direction_) {
        return;
    }
    
    // 已经收集满10个，不再收集
    if (direction_count_ >= 10) {
        return;
    }
    
    // 解析方向并存入数组
    if (msg->data == "left") {
        direction_history_[direction_count_] = 1;
        direction_count_++;
        ROS_INFO("Direction collected [%d/10]: LEFT", direction_count_);
    } else if (msg->data == "right") {
        direction_history_[direction_count_] = -1;
        direction_count_++;
        ROS_INFO("Direction collected [%d/10]: RIGHT", direction_count_);
    }
}

void Session4WaitDirection::ZebraCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
    // 遍历 targets 查找 zebra
    for (const auto& target : msg->targets) {
        if (target.obj_class == "zebra") {
            // 检测到 zebra
            if (zebra_detected_.load() == 0) {
                zebra_detected_.store(1);
                ROS_INFO("Zebra detected! Starting approach...");
            }
            // 更新最后一次检测到 zebra 的时间
            last_zebra_time_ = ros::Time::now();
            return;
        }
    }
}

void Session4WaitDirection::CheckZebraDisappear() {
    // 如果之前检测到过 zebra，但现在已经超过一定时间没有检测到
    // 说明 zebra 消失（车辆已经靠近，摄像头看不到了）
    if (zebra_detected_.load() == 1) {
        double time_since_last_zebra = (ros::Time::now() - last_zebra_time_).toSec();
        if (time_since_last_zebra > zebra_disappear_timeout_) {
            zebra_detected_.store(2);
            ROS_INFO("Zebra disappeared! Close enough, preparing to stop...");
        }
    }
}

void Session4WaitDirection::PointsCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
    if (!msg->lane_points.empty() && !msg->lane_points[0].points.empty() &&
        msg->lane_points[0].points[0].point.size() >= 3) {
        left_x_[msg->image_seq] = msg->lane_points[0].points[0].point[1].x;
        right_x_[msg->image_seq] = msg->lane_points[0].points[0].point[2].x;
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    point_queue_.push(msg);
    if (point_queue_.size() > 1) {
        point_queue_.pop();
    }
}

void Session4WaitDirection::PublishStop() {
    ros_serial::to32 msg;
    msg.speed = 0.0;
    msg.angle = 0.0;
    msg.run_flag = false;
    publisher_.publish(msg);
}

void Session4WaitDirection::Execute() {
    // 检查 zebra 是否消失
    CheckZebraDisappear();
    
    int zebra_state = zebra_detected_.load();
    
    // 状态 0 或 1：还没到达停车位置，继续巡线
    if (zebra_state < 2) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!point_queue_.empty()) {
            auto point_msg = point_queue_.top();
            lock.unlock();
            
            if (!point_msg->targets.empty()) {
                LineFollowing(point_msg->targets[0]);
            }
            
            lock.lock();
            point_queue_.pop();
        }
        return;
    }
    
    // 状态 2：zebra 消失，开始停车等待
    if (!started_) {
        start_time_ = ros::Time::now();
        started_ = true;
        ROS_INFO("Session 4: Stopped at zebra crossing, waiting for direction...");
        
        // 开始收集方向
        collecting_direction_ = true;
        direction_count_ = 0;
        for (int i = 0; i < 10; i++) {
            direction_history_[i] = 0;
        }
        ROS_INFO("Started collecting direction signals...");
    }
    
    // 停车循环，持续 11 秒
    ros::Rate rate(30);  // 30Hz
    double elapsed = (ros::Time::now() - start_time_).toSec();
    
    while (elapsed < 11.0 && ros::ok()) {
        ros::spinOnce();
        
        // 持续发布停车指令
        PublishStop();
        
        // 检索指示牌方向（收集满10个后判断）
        if (direction_count_ >= 10 && direction_.load() == 0) {
            int judged_direction = JudgeDirection();
            if (judged_direction != 0) {
                direction_.store(judged_direction);
                ROS_INFO("Direction judged from %d samples: %s", 
                         direction_count_, judged_direction > 0 ? "LEFT" : "RIGHT");
            }
        }
        
        elapsed = (ros::Time::now() - start_time_).toSec();
        ROS_INFO_THROTTLE(2, "Session 4: Waiting for direction... (%.1fs / 11.0s)", elapsed);
        
        rate.sleep();
    }
    
    // 退出循环，11秒已到
    ROS_INFO("Session 4: 11 seconds elapsed, stopping collection.");
    collecting_direction_ = false;
    
    // 如果还没确定方向，强制判断
    if (direction_.load() == 0) {
        int judged_direction = JudgeDirection();
        direction_.store(judged_direction);
        ROS_INFO("Final direction (from %d samples): %s", 
                 direction_count_, judged_direction > 0 ? "LEFT" : "RIGHT");
    }
}

bool Session4WaitDirection::HasDirection() const {
    return direction_.load() != 0;
}

int Session4WaitDirection::GetDirection() const {
    int dir = direction_.load();
    if (dir != 0) {
        return dir;
    }
    // 超时返回默认方向
    if (IsTimeout()) {
        return (default_direction_ > 0) ? 1 : -1;
    }
    return 0;
}

int Session4WaitDirection::JudgeDirection() const {
    int sum = 0;
    for (int i = 0; i < direction_count_; i++) {
        sum += direction_history_[i];  // 1=left, -1=right
    }
    
    ROS_INFO("Direction sum: %d (left=%d, right=%d)", 
             sum, (direction_count_ + sum) / 2, (direction_count_ - sum) / 2);
    
    if (sum > 0) return 1;       // 左转多
    else if (sum < 0) return -1; // 右转多
    else return default_direction_ > 0 ? 1 : -1;  // 平局用默认
}

bool Session4WaitDirection::IsTimeout() const {
    if (!started_) return false;
    double elapsed = (ros::Time::now() - start_time_).toSec();
    return elapsed >= wait_timeout_;
}

bool Session4WaitDirection::IsComplete() const {
    // 已停车且（收到方向 或 超时）
    return started_ && (HasDirection() || IsTimeout());
}

void Session4WaitDirection::LineFollowing(const obstacle_detector::Target& point_msg, float add_speed) {
    ros_serial::to32 to32_msg;
    float angle = 0.0;

    // ========== 优先处理回正角度队列 ==========
    if (!correctAngle_.empty()) {
        iscorrecting_ = true;
        angle = correctAngle_.front();
        correctAngle_.pop();

        ROS_WARN("Correcting: angle=%.1f, remaining_steps=%zu", angle, correctAngle_.size());

        if (correctAngle_.empty()) {
            iscorrecting_ = false;
            ROS_INFO("Correction complete!");
        }
    }
    // ========== 正常巡线 ==========
    else {
        ROS_INFO("LineFollowing...");

        // 安全检查
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
            angle = last_angle_ * 0.8;  // 使用上一帧角度的80%，逐渐减弱
            angle = std::max(-10.0f, std::min(10.0f, angle));
            
            to32_msg.speed = lineSpeed_ * 0.7;  // 降速行驶
            to32_msg.angle = angle;
            to32_msg.run_flag = true;
            publisher_.publish(to32_msg);
            return;
        }

        float error = mid - 350.0;
        
        // 对 error 进行滑动均值滤波
        float filtered_error = ApplyMovingAverage(error_buffer_, error);
        ROS_INFO("error: raw=%.1f, filtered=%.1f", error, filtered_error);
        
        // 计算角度
        angle = -1.0 * follow_p_ * filtered_error;
        angle = std::max(-10.0f, std::min(10.0f, angle));
        
        // 检查角度变化是否异常
        if (!IsAngleValid(angle)) {
            ROS_WARN("Angle change too large (current=%.1f, last=%.1f), skipping frame", angle, last_angle_);
            return;
        }
        
        // 对 angle 进行滑动均值滤波
        angle = ApplyMovingAverage(angle_buffer_, angle);
        last_angle_ = angle;
        first_frame_ = false;
        
        ROS_INFO("target_angle: %.1f", angle);
    }

    to32_msg.speed = lineSpeed_ + add_speed;
    to32_msg.angle = angle;
    to32_msg.run_flag = true;
    publisher_.publish(to32_msg);
}

// 滑动均值滤波器
float Session4WaitDirection::ApplyMovingAverage(std::deque<float>& buffer, float new_value) {
    buffer.push_back(new_value);
    
    if (buffer.size() > filter_window_size_) {
        buffer.pop_front();
    }
    
    float sum = 0.0;
    for (float val : buffer) {
        sum += val;
    }
    
    return sum / buffer.size();
}

// 检查角度变化是否有效
bool Session4WaitDirection::IsAngleValid(float current_angle) {
    if (first_frame_) {
        return true;
    }
    float diff = std::abs(current_angle - last_angle_);
    return diff <= max_angle_diff_;
}

// 测试用 main 函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "session4_wait_direction_test");
    
    Session4WaitDirection session;
    ros::Rate rate(30);  // 30Hz
    
    ROS_INFO("=== Session 4 Test: Line Following + Zebra Detection + Wait for Direction ===");
    ROS_INFO("Flow: LineFollow -> Detect Zebra -> Zebra Disappear -> Stop -> Wait Direction");
    
    while (ros::ok()) {
        ros::spinOnce();
        session.Execute();
        
        // 检查是否完成
        if (session.IsComplete()) {
            int dir = session.GetDirection();
            if (session.HasDirection()) {
                ROS_INFO("Direction confirmed: %s", dir == 1 ? "LEFT" : "RIGHT");
            } else {
                ROS_WARN("Timeout! Using default direction: %s", dir == 1 ? "LEFT" : "RIGHT");
            }
            break;
        }
        
        rate.sleep();
    }
    
    ROS_INFO("Session 4 complete! Direction: %s", 
             session.GetDirection() == 1 ? "LEFT" : "RIGHT");
    return 0;
}
