#include "racing_control_test.h"
#include <algorithm>

/**
 * Session 2: 巡线 + 避障
 * Mode 2 - 跟随车道线行驶，遇到障碍物时通过调整目标中心点避开
 */

Session2LineAvoid::Session2LineAvoid() {
    point_subscriber_ = nh_.subscribe("/line_detector/center", 10,
                                       &Session2LineAvoid::PointCallback, this);
    target_subscriber_ = nh_.subscribe("/obstacle/detection", 10,
                                        &Session2LineAvoid::TargetCallback, this);
    publisher_ = nh_.advertise<ros_serial::to32>("/cmd_vel", 5);

    nh_.getParam("avoidBottom", avoidBottom_);
    nh_.getParam("lineSpeed", lineSpeed_);
    nh_.getParam("follow_p", follow_p_);
    nh_.getParam("avoid_p", avoid_p_);  // 保留以兼容，但不再使用

    int filter_window_tmp = static_cast<int>(filter_window_size_);
    nh_.getParam("filter_window_size", filter_window_tmp);
    filter_window_size_ = static_cast<size_t>(filter_window_tmp);

    nh_.getParam("max_angle_diff", max_angle_diff_);

    adjusted_center_ = 350.0;  // 初始化默认中心点

    ROS_INFO("Session2LineAvoid initialized!");
    ROS_INFO("Parameters: avoidBottom=%d, lineSpeed=%.2f, filter_window=%zu, max_angle_diff=%.1f",
             avoidBottom_, lineSpeed_, filter_window_size_, max_angle_diff_);
}

Session2LineAvoid::~Session2LineAvoid() {
    Stop();
}

void Session2LineAvoid::Stop() {
    process_stop_ = true;
    if (process_thread_ && process_thread_->joinable()) {
        process_thread_->join();
    }
}

void Session2LineAvoid::PointCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
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

void Session2LineAvoid::TargetCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
    sub_target_ = true;
    std::unique_lock<std::mutex> lock(mutex_);
    targets_queue_.push(msg);
    if (targets_queue_.size() > 1) {
        targets_queue_.pop();
    }
}

void Session2LineAvoid::Execute() {
    std::unique_lock<std::mutex> lock(mutex_);

    // ========== 第一步：处理障碍物检测，更新 adjusted_center_ ==========
    if (!targets_queue_.empty() && sub_target_) {
        auto target_msg = targets_queue_.top();
        lock.unlock();

        bool obstacleFound = false;
        obstacle_detector::PerceptionTarget currentTarget;

        if (!target_msg->obstacles.empty()) {
            for (auto& target : target_msg->obstacles) {
                if (target.obj_class == "b_obs") {

                    // 情况1: 未在避障中，检测到障碍物进入避障区域
                    if (!isAvoiding_ && target.y2 >= avoidBottom_) {
                        ROS_WARN("Obstacle detected! Starting avoidance by adjusting center...");
                        isAvoiding_ = true;
                        avoidingClass_ = target.obj_class;
                        noObstacleCount_ = 0;
                        obstacleFound = true;
                        currentTarget = target;

                        // 计算并设置 adjusted_center_
                        UpdateAdjustedCenter(currentTarget);
                        break;
                    }

                    // 情况2: 正在避障中，障碍物仍在画面中
                    if (isAvoiding_ && target.obj_class == avoidingClass_) {
                        obstacleFound = true;
                        currentTarget = target;
                        noObstacleCount_ = 0;

                        // 持续更新 adjusted_center_
                        UpdateAdjustedCenter(currentTarget);
                        break;
                    }
                }
            }
        }

        // ========== 第二步：处理避障状态转换 ==========
        if (isAvoiding_) {
            if (!obstacleFound) {
                // 障碍物未检测到，增加计数
                noObstacleCount_++;
                ROS_INFO("Obstacle not detected, count: %d/%d", noObstacleCount_, noObstacleThreshold_);

                if (noObstacleCount_ >= noObstacleThreshold_) {
                    // 障碍物消失，准备回正
                    ROS_WARN("Obstacle disappeared! Starting correction...");

                    // 取消平滑分步回正，直接按当前偏移量一次性回正
                    float current_offset = adjusted_center_ - 350.0;
                    if (std::abs(current_offset) > 1e-3) {
                        correctAngle_.push(-current_offset);
                    }

                    isAvoiding_ = false;
                    avoidingClass_ = "";
                    noObstacleCount_ = 0;
                    // 不立即重置 adjusted_center_，在 LineFollowing 中平滑过渡
                }
            }
        }

        lock.lock();
        targets_queue_.pop();
    }

    // ========== 第三步：统一执行巡线（包含避障和回正）==========
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        if (!point_msg->lane_points.empty()) {
            LineFollowing(point_msg->lane_points[0]);
        }
        lock.lock();
        point_queue_.pop();
    }
}

void Session2LineAvoid::UpdateAdjustedCenter(const obstacle_detector::PerceptionTarget& target) {
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        if (!point_msg->lane_points.empty() &&
            !point_msg->lane_points[0].points.empty() &&
            point_msg->lane_points[0].points[0].point.size() >= 3) {

            float left = point_msg->lane_points[0].points[0].point[1].x;
            float right = point_msg->lane_points[0].points[0].point[2].x;
            float obstacle_center = (target.x1 + target.x2) / 2.0f;

            // 计算障碍物相对位置
            float left_dist = obstacle_center - left;
            float right_dist = right - obstacle_center;

            ROS_INFO("Lane: left=%.1f, right=%.1f, obstacle_center=%.1f", left, right, obstacle_center);
            ROS_INFO("Distance: left_dist=%.1f, right_dist=%.1f", left_dist, right_dist);

            // 计算目标中心点偏移
            float target_offset = 0.0;
            if (left_dist < right_dist) {
                // 障碍物偏左，目标点向右偏移
                target_offset = -35.0;  // 减小 x 值 = 向右
                judgeLeft_ = false;
                ROS_WARN("Obstacle on LEFT, shifting target RIGHT (offset=%.1f)", target_offset);
            } else {
                // 障碍物偏右，目标点向左偏移
                target_offset = 35.0;   // 增大 x 值 = 向左
                judgeLeft_ = true;
                ROS_WARN("Obstacle on RIGHT, shifting target LEFT (offset=%.1f)", target_offset);
            }

            // 取消平滑，直接使用目标中心
            float target_center = 350.0 + target_offset;
            adjusted_center_ = target_center;

            ROS_INFO("Updated adjusted_center_ to %.1f (no smoothing)", adjusted_center_);
        }
    }
}

void Session2LineAvoid::LineFollowing(const obstacle_detector::Target& point_msg, float add_speed) {
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
            adjusted_center_ = 350.0;  // 回正完成，重置中心点
            ROS_INFO("Correction complete!");
        }
    }
    // ========== 正常巡线（包含避障）==========
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

        // 使用当前的 adjusted_center_（避障时已动态调整）
        float error = mid - adjusted_center_;

        if (isAvoiding_) {
            ROS_INFO("AVOIDING: mid=%.1f, target=%.1f, error=%.1f",
                     mid, adjusted_center_, error);
        } else {
            ROS_INFO("NORMAL: mid=%.1f, error=%.1f", mid, error);
        }

        // 对 error 进行滑动均值滤波
        float filtered_error = ApplyMovingAverage(error_buffer_, error);
        ROS_INFO("error: raw=%.1f, filtered=%.1f", error, filtered_error);

        // 计算角度（统一使用 follow_p_）
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
float Session2LineAvoid::ApplyMovingAverage(std::deque<float>& buffer, float new_value) {
    buffer.push_back(new_value);

    // 保持窗口大小
    if (buffer.size() > filter_window_size_) {
        buffer.pop_front();
    }

    // 计算均值
    float sum = 0.0;
    for (float val : buffer) {
        sum += val;
    }

    return sum / buffer.size();
}

// 检查角度变化是否有效（不是异常跳变）
bool Session2LineAvoid::IsAngleValid(float current_angle) {
    // 第一帧总是有效
    if (first_frame_) {
        return true;
    }

    // 检查与上一帧的差异
    float diff = std::abs(current_angle - last_angle_);
    return diff <= max_angle_diff_;
}

// 测试用 main 函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "session2_line_avoid_test");

    Session2LineAvoid session;
    ros::Rate rate(30);

    ROS_INFO("=== Session 2 Test: Line Following + Obstacle Avoidance ===");
    ROS_INFO("Press Ctrl+C to stop");

    while (ros::ok()) {
        ros::spinOnce();
        session.Execute();
        rate.sleep();
    }

    ROS_INFO("Session 2 test stopped.");
    return 0;
}
