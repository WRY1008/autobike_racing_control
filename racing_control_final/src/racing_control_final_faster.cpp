#include "racing_control_final.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * Racing Control Final: 整合版控制节点
 * 
 * 状态机流程：
 * ┌────────────────────────────────────────────────────────────────┐
 * │  SESSION_2: 巡线 + 避障 + 减速带                               │
 * │  跳变条件: 连续检测到3帧斑马线 → SESSION_4                      │
 * └────────────────────────────────────────────────────────────────┘
 *                               ↓
 * ┌────────────────────────────────────────────────────────────────┐
 * │  SESSION_4: 巡线 → 斑马线消失 → 停车11秒 → 收集方向             │
 * │  跳变条件: 停车超过11秒 且 已收到方向 → SESSION_5               │
 * └────────────────────────────────────────────────────────────────┘
 *                               ↓
 * ┌────────────────────────────────────────────────────────────────┐
 * │  SESSION_5: 定时转向 → 回正 → 巡线 → 检测y_obs → 可能重复转向   │
 * │  跳变条件: 完成所有转向流程 → SESSION_COMPLETE                  │
 * └────────────────────────────────────────────────────────────────┘
 */

RacingControlFinal::RacingControlFinal() {
    point_subscriber_ = nh_.subscribe("/line_detector/center", 10,
                                       &RacingControlFinal::PointCallback, this);
    target_subscriber_ = nh_.subscribe("/obstacle/detection", 10,
                                        &RacingControlFinal::TargetCallback, this);
    direction_subscriber_ = nh_.subscribe("/yolo_fastestv2/direction", 1,
                                           &RacingControlFinal::DirectionCallback, this);
    publisher_ = nh_.advertise<ros_serial::to32>("/cmd_vel", 5);

    // 读取参数
    nh_.getParam("avoidBottom", avoidBottom_);
    nh_.getParam("lineSpeed", lineSpeed_);
    nh_.getParam("follow_p", follow_p_);
    nh_.getParam("avoid_p", avoid_p_);
    nh_.getParam("LeftOrRight", default_direction_);

    int filter_window_tmp = static_cast<int>(filter_window_size_);
    nh_.getParam("filter_window_size", filter_window_tmp);
    filter_window_size_ = static_cast<size_t>(filter_window_tmp);
    nh_.getParam("max_angle_diff", max_angle_diff_);

    nh_.getParam("wait_timeout", wait_timeout_);
    nh_.getParam("zebra_disappear_timeout", zebra_disappear_timeout_);

    nh_.getParam("turn_angle_left", turn_angle_left_);
    nh_.getParam("turn_angle_right", turn_angle_right_);
    nh_.getParam("turn_duration_left", turn_duration_left_);
    nh_.getParam("turn_duration_right", turn_duration_right_);
    nh_.getParam("correct_duration_left", correct_duration_left_);
    nh_.getParam("correct_duration_right", correct_duration_right_);
    nh_.getParam("turn2_angle_left", turn2_angle_left_);
    nh_.getParam("turn2_angle_right", turn2_angle_right_);
    nh_.getParam("turn2_duration_left", turn2_duration_left_);
    nh_.getParam("turn2_duration_right", turn2_duration_right_);
    nh_.getParam("correct2_duration_left", correct2_duration_left_);
    nh_.getParam("correct2_duration_right", correct2_duration_right_);

    // y_obs_wait_timeout_：从“秒”改为“帧数”语义。
    // 优先读取 y_obs_wait_frames；若不存在则兼容旧键 y_obs_wait_timeout（可能是 int/double）。
    {
        int frames_tmp = y_obs_wait_timeout_;
        if (nh_.getParam("y_obs_wait_frames", frames_tmp)) {
            y_obs_wait_timeout_ = std::max(1, frames_tmp);
        } else if (nh_.getParam("y_obs_wait_timeout", frames_tmp)) {
            y_obs_wait_timeout_ = std::max(1, frames_tmp);
        } else {
            double legacy_double = 0.0;
            if (nh_.getParam("y_obs_wait_timeout", legacy_double)) {
                y_obs_wait_timeout_ = std::max(1, static_cast<int>(std::lround(legacy_double)));
            }
        }
    }

    // 读取赛道恢复参数
    nh_.getParam("abnormal_threshold", abnormal_threshold_);
    nh_.getParam("recovery_duration", recovery_duration_);

    // 指示牌中心偏移缩放系数（可选参数；没设置则用默认值 turn_center_coeff_）
    nh_.getParam("turn_center_coeff", turn_center_coeff_);

    // 是否启用“指示牌中心动态转向”（可选参数；没设置则用默认值 enable_dynamic_turn_from_sign_）
    nh_.getParam("use_dynamic_turn_from_sign", enable_dynamic_turn_from_sign_);

    // zebra 框底边 y2 达到该阈值后，开始收集 turn_left/turn_right 方向（可选参数）
    {
        double tmp = static_cast<double>(zebra_direction_start_bottom);
        if (nh_.getParam("zebra_direction_start_bottom", tmp)) {
            zebra_direction_start_bottom = static_cast<float>(tmp);
        }
        ROS_INFO("zebra_direction_start_bottom: %.1f", zebra_direction_start_bottom);
    }

    // 读取黄桶偏移参数
    nh_.getParam("y_obs_offset", y_obs_offset_);
    ROS_INFO("y_obs_offset: %.1f", y_obs_offset_);

    last_zebra_time_ = ros::Time::now();

    ROS_INFO("RacingControlFinal initialized!");
    ROS_INFO("Parameters: lineSpeed=%.2f, avoidBottom=%d, wait_timeout=%.1fs", 
             lineSpeed_, avoidBottom_, wait_timeout_);
    ROS_INFO("Turn params: left_angle=%.1f, right_angle=%.1f, left_dur=%.1fs, right_dur=%.1fs",
             turn_angle_left_, turn_angle_right_, turn_duration_left_, turn_duration_right_);
    ROS_INFO("Turn center coeff: turn_center_coeff=%.3f (0.0 => no scaling)", turn_center_coeff_);
    ROS_INFO("Dynamic turn-from-sign enabled: %s (param: use_dynamic_turn_from_sign)",
             enable_dynamic_turn_from_sign_ ? "true" : "false");
}

RacingControlFinal::~RacingControlFinal() {
    Stop();
}

void RacingControlFinal::Stop() {
    process_stop_ = true;
    if (process_thread_ && process_thread_->joinable()) {
        process_thread_->join();
    }
}

// ==================== 回调函数 ====================

void RacingControlFinal::PointCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
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

void RacingControlFinal::TargetCallback(const obstacle_detector::PerceptionTargets::ConstPtr& msg) {
    sub_target_ = true;
    
    // ========== Session 2: 检测斑马线用于跳变 ==========
    if (current_state_ == SessionState::SESSION_2) {
        bool zebra_found = false;
        for (const auto& target : msg->obstacles) {
            if (target.obj_class == "zebra") {
                // 远距离误检过滤：只有当斑马线检测框底边 y2 到达阈值才算有效
                if (static_cast<int>(target.y2) >= zebra_valid_bottom_y_) {
                    zebra_found = true;
                    break;
                }
            }
        }
        
        if (zebra_found) {
            zebra_detect_count_++;
            ROS_INFO("Zebra detected in Session 2, count: %d/%d", 
                     zebra_detect_count_, zebra_detect_threshold_);
        } else {
            zebra_detect_count_ = 0;  // 重置计数
        }
    }
    
    // ========== Session 4: 检测斑马线状态 ==========
    if (current_state_ == SessionState::SESSION_4) {
        bool zebra_found = false;
        float zebra_best_bottom = -1.0f;
        for (const auto& target : msg->obstacles) {
            if (target.obj_class == "zebra") {
                zebra_found = true;
                zebra_best_bottom = std::max(zebra_best_bottom, static_cast<float>(target.y2));
            }
        }

        if (zebra_found) {
            if (zebra_detected_.load() == 0) {
                zebra_detected_.store(1);
                ROS_INFO("Session 4: Zebra detected! Starting approach...");
            }
            last_zebra_time_ = ros::Time::now();

            // 关键：当 zebra 检测框底边 y2 达到阈值时，开启方向收集
            if (!collecting_direction_ && zebra_best_bottom >= zebra_direction_start_bottom) {
                collecting_direction_ = true;
                ROS_INFO("Session 4: Start collecting direction (zebra y2=%.1f >= %.1f)",
                         zebra_best_bottom, zebra_direction_start_bottom);
            }
        }

        // 在停车后收集方向期间：同步记录 turn_left/turn_right 的检测框中心 x
        // 用于后续根据指示牌位置动态缩放转向角度/时间
        if (collecting_direction_) {
            float best_left_score = -1.0f;
            float best_right_score = -1.0f;
            float best_left_cx = 320.0f;
            float best_right_cx = 320.0f;

            for (const auto& target : msg->obstacles) {
                if (target.obj_class == "turn_left") {
                    if (target.score > best_left_score) {
                        best_left_score = target.score;
                        best_left_cx = 0.5f * (static_cast<float>(target.x1) + static_cast<float>(target.x2));
                    }
                } else if (target.obj_class == "turn_right") {
                    if (target.score > best_right_score) {
                        best_right_score = target.score;
                        best_right_cx = 0.5f * (static_cast<float>(target.x1) + static_cast<float>(target.x2));
                    }
                }
            }

            if (best_left_score >= 0.0f || best_right_score >= 0.0f) {
                float chosen_cx = (best_left_score > best_right_score) ? best_left_cx : best_right_cx;
                latest_sign_center_x_ = chosen_cx;
                latest_sign_center_valid_ = true;
                sign_center_sum_x_ += chosen_cx;
                sign_center_count_++;
                ROS_INFO_THROTTLE(1, "Session 4: sign center x=%.1f (samples=%d)",
                                  latest_sign_center_x_, sign_center_count_);
            }
        }
    }
    
    // ========== Session 5: 检测y_obs ==========
    if (current_state_ == SessionState::SESSION_5) {
        session5_last_target_seq_.store(msg->image_seq, std::memory_order_relaxed);

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
                y_obs_detect_count_ = 0;
            }
        } else if (!current_frame_detected) {
            if (y_obs_detect_count_ > 0) {
                ROS_INFO("Y_OBS detection interrupted, resetting count");
            }
            y_obs_detect_count_ = 0;
        }
    }

    std::unique_lock<std::mutex> lock(mutex_);
    targets_queue_.push(msg);
    if (targets_queue_.size() > 1) {
        targets_queue_.pop();
    }
}

void RacingControlFinal::DirectionCallback(const std_msgs::String::ConstPtr& msg) {
    // 只有在 Session 4 且 zebra 框底边 y2 达到阈值后才开始收集方向
    if (!collecting_direction_) {
        return;
    }
    
    if (direction_count_ >= 10) {
        return;
    }
    
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

// ==================== 主执行函数 ====================

void RacingControlFinal::Execute() {
    switch (current_state_) {
        case SessionState::SESSION_2:
            ExecuteSession2();
            
            // 检查跳变条件：连续3帧检测到斑马线
            if (zebra_detect_count_ >= zebra_detect_threshold_) {
                ROS_WARN("========== SESSION 2 → SESSION 4 ==========");
                ROS_INFO("Zebra detected for %d consecutive frames, switching to Session 4", 
                         zebra_detect_count_);
                current_state_ = SessionState::SESSION_4;
                zebra_detect_count_ = 0;
                zebra_detected_.store(1);  // 已经检测到斑马线
                last_zebra_time_ = ros::Time::now();

                // 一进入 Session 4 就开始收集指示牌方向/中心点。
                // 目的：解决“停车后相机扫不到指示牌”的情况，保留最后一次有效中心点用于后续计算。
                // 改为：等待 zebra 检测框底边 y2 达到 zebra_direction_start_bottom 后再开启收集。
                collecting_direction_ = false;
                direction_.store(0);
                direction_count_ = 0;
                for (int i = 0; i < 10; i++) {
                    direction_history_[i] = 0;
                }
                sign_center_sum_x_ = 0.0f;
                sign_center_count_ = 0;
                latest_sign_center_x_ = 350.0f;
                latest_sign_center_valid_ = false;
                use_dynamic_turn_from_sign_ = false;
                stop_started_ = false;
            }
            break;

        case SessionState::SESSION_4:
            ExecuteSession4();
            
            // 检查跳变条件：停车超过11秒 且 已收到方向
            if (stop_started_) {
                double elapsed = (ros::Time::now() - stop_start_time_).toSec();
                if (elapsed >= wait_timeout_ && direction_.load() != 0) {
                    ROS_WARN("========== SESSION 4 → SESSION 5 ==========");
                    ROS_INFO("Waited %.1fs, direction=%s, switching to Session 5", 
                             elapsed, direction_.load() > 0 ? "LEFT" : "RIGHT");
                    current_state_ = SessionState::SESSION_5;
                    
                    // 初始化 Session 5
                    original_direction_ = direction_.load();
                    turn_count_ = 0;
                    y_obs_flag_ = 0;
                    y_obs_detected_ = false;
                    y_obs_detect_count_ = 0;
                    session5_complete_ = false;
                }
            }
            break;

        case SessionState::SESSION_5:
            ExecuteSession5();
            
            // 检查跳变条件：Session 5 完成
            if (session5_complete_) {
                ROS_WARN("========== SESSION 5 → SESSION COMPLETE ==========");
                ROS_INFO("All turn maneuvers complete!");
                current_state_ = SessionState::SESSION_COMPLETE;
            }
            break;

        case SessionState::SESSION_COMPLETE:
            // 完成后持续巡线，使用冲刺速度
            ProcessLineAndObstacle(final_sprint_speed_);
            break;
    }
}

// ==================== Session 2: 巡线 + 避障 + 减速带 ====================

void RacingControlFinal::ExecuteSession2() {
    std::unique_lock<std::mutex> lock(mutex_);

    // ========== 第一步：处理障碍物检测，更新 adjusted_center_ ==========
    if (!targets_queue_.empty() && sub_target_) {
        auto target_msg = targets_queue_.top();
        lock.unlock();

        bool obstacleFound = false;
        bool slowFound = false;
        obstacle_detector::PerceptionTarget currentTarget;

        if (!target_msg->obstacles.empty()) {
            for (auto& target : target_msg->obstacles) {
                // 检测减速带
                if (target.obj_class == "slow") {
                    slowFound = true;
                    
                    if (slow_flag_ != 1) {
                        slow_flag_ = 1;
                        isSlowing_ = true;
                        slow_last_time_ = ros::Time::now();
                        ROS_WARN("Speed bump detected! Increasing speed (flag: %d -> 1)", slow_flag_);
                    }
                }
                
                // 检测障碍物
                if (target.obj_class == "b_obs") {
                    if (!isAvoiding_ && target.y2 >= avoidBottom_) {
                        ROS_WARN("Obstacle detected! Starting avoidance by adjusting center...");
                        isAvoiding_ = true;
                        avoidingClass_ = target.obj_class;
                        noObstacleCount_ = 0;
                        obstacleFound = true;
                        currentTarget = target;
                        UpdateAdjustedCenter(currentTarget);
                        break;
                    }

                    if (isAvoiding_ && target.obj_class == avoidingClass_) {
                        obstacleFound = true;
                        currentTarget = target;
                        noObstacleCount_ = 0;
                        UpdateAdjustedCenter(currentTarget);
                        break;
                    }
                }
            }
        }
        
        // 处理减速带状态转换
        if (slow_flag_ == 1 && !slowFound) {
            slow_flag_ = 2;
            slow_last_time_ = ros::Time::now();
            ROS_INFO("Speed bump disappeared, keeping increased speed (flag: 1 -> 2)");
        }
        
        if (slow_flag_ == 2) {
            double elapsed = (ros::Time::now() - slow_last_time_).toSec();
            if (elapsed >= time_extend_) {
                slow_flag_ = 0;
                isSlowing_ = false;
                ROS_INFO("Extended speed period ended (%.1fs), returning to normal speed", elapsed);
            }
        }

        // 处理避障状态转换
        if (isAvoiding_) {
            if (!obstacleFound) {
                noObstacleCount_++;
                ROS_INFO("Obstacle not detected, count: %d/%d", noObstacleCount_, noObstacleThreshold_);

                if (noObstacleCount_ >= noObstacleThreshold_) {
                    ROS_WARN("Obstacle disappeared! Resetting to center...");

                    // 直接重置中心点，不使用回正队列
                    adjusted_center_ = 320.0;
                    isAvoiding_ = false;
                    avoidingClass_ = "";
                    noObstacleCount_ = 0;
                }
            }
        }

        lock.lock();
        targets_queue_.pop();
    }

    // ========== 第二步：统一执行巡线 ==========
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        if (!point_msg->lane_points.empty()) {
            float speed_adjustment = isSlowing_ ? 0.5 : 0.0;
            LineFollowing(point_msg->lane_points[0], speed_adjustment);
        }
        lock.lock();
        point_queue_.pop();
    }
}

// ==================== Session 4: 斑马线停车 + 等待方向 ====================

void RacingControlFinal::ExecuteSession4() {
    // 检查 zebra 是否消失
    CheckZebraDisappear();
    
    int zebra_state = zebra_detected_.load();
    
    // 状态 0 或 1：还没到达停车位置，继续巡线
    if (zebra_state < 2) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!point_queue_.empty()) {
            auto point_msg = point_queue_.top();
            lock.unlock();
            
            if (!point_msg->lane_points.empty()) {
                LineFollowing(point_msg->lane_points[0]);
            }
            
            lock.lock();
            point_queue_.pop();
        }
        return;
    }
    
    // 状态 2：zebra 消失，开始停车等待
    if (!stop_started_) {
        stop_start_time_ = ros::Time::now();
        stop_started_ = true;
        ROS_INFO("Session 4: Stopped at zebra crossing, waiting for direction...");

        // 注意：不在停车时清空方向/中心点缓存。
        // 因为指示牌可能在停车前就已经离开视野，我们需要保留“最后一次看到时”的中心点。
        ROS_INFO("Session 4: Stopped. Keep collecting_direction_=%s (start condition: zebra y2>=%.1f)",
                 collecting_direction_ ? "true" : "false", zebra_direction_start_bottom);
    }
    
    // 持续发布停车指令
    PublishStop();
    
    // 检索指示牌方向（只要收集到数据就进行判断）
    if (direction_count_ > 0 && direction_.load() == 0) {
        int judged_direction = JudgeDirection();
        if (judged_direction != 0) {
            direction_.store(judged_direction);
            ROS_INFO("Direction judged from %d samples: %s", 
                     direction_count_, judged_direction > 0 ? "LEFT" : "RIGHT");

            // 方向一旦确定：根据指示牌中心位置计算“第一次转向”的动态角度/时间
            if (!enable_dynamic_turn_from_sign_) {
                use_dynamic_turn_from_sign_ = false;
                ROS_WARN("Dynamic turn-from-sign disabled by param; using base turn params.");
                return;
            }

            const bool has_sign_center = (sign_center_count_ > 0) || latest_sign_center_valid_;
            float cx = 350.0f;
            if (sign_center_count_ > 0) {
                cx = sign_center_sum_x_ / static_cast<float>(sign_center_count_);
            } else if (latest_sign_center_valid_) {
                cx = latest_sign_center_x_;
            }

            if (!has_sign_center) {
                // 超时/判定时可能根本没检测到指示牌：这里明确回退到原版参数（默认值）。
                turn_scale_from_sign_ = 1.0f;
                turn_angle_from_sign_ = (judged_direction > 0) ? turn_angle_left_ : turn_angle_right_;
                turn_duration_from_sign_ = (judged_direction > 0) ? turn_duration_left_ : turn_duration_right_;
                use_dynamic_turn_from_sign_ = false;
                ROS_WARN("No sign center collected; fallback to base turn params: angle=%.2f, duration=%.2f",
                         turn_angle_from_sign_, turn_duration_from_sign_);
            } else {
                ComputeTurnFromSignCenter(judged_direction, cx, turn_angle_from_sign_, turn_duration_from_sign_, turn_scale_from_sign_);
                use_dynamic_turn_from_sign_ = true;
                ROS_WARN("Dynamic turn from sign: dir=%s, cx=%.1f => scale=%.3f, angle=%.2f, duration=%.2f",
                         judged_direction > 0 ? "LEFT" : "RIGHT", cx, turn_scale_from_sign_, turn_angle_from_sign_, turn_duration_from_sign_);
            }
        }
    }
    
    double elapsed = (ros::Time::now() - stop_start_time_).toSec();
    ROS_INFO_THROTTLE(2, "Session 4: Waiting for direction... (%.1fs / %.1fs)", elapsed, wait_timeout_);
    
    // 11秒后强制判断方向
    if (elapsed >= wait_timeout_ && direction_.load() == 0) {
        collecting_direction_ = false;
        int judged_direction = JudgeDirection();
        direction_.store(judged_direction);
        ROS_INFO("Final direction (from %d samples): %s", 
                 direction_count_, judged_direction > 0 ? "LEFT" : "RIGHT");

        // 超时后最终方向也需要计算动态角度/时间
        if (!enable_dynamic_turn_from_sign_) {
            use_dynamic_turn_from_sign_ = false;
            ROS_WARN("Dynamic turn-from-sign disabled by param (timeout path); using base turn params.");
            return;
        }

        const bool has_sign_center = (sign_center_count_ > 0) || latest_sign_center_valid_;
        float cx = 350.0f;
        if (sign_center_count_ > 0) {
            cx = sign_center_sum_x_ / static_cast<float>(sign_center_count_);
        } else if (latest_sign_center_valid_) {
            cx = latest_sign_center_x_;
        }

        if (!has_sign_center) {
            // 超时且没检测到指示牌：默认使用原版参数。
            turn_scale_from_sign_ = 1.0f;
            turn_angle_from_sign_ = (judged_direction > 0) ? turn_angle_left_ : turn_angle_right_;
            turn_duration_from_sign_ = (judged_direction > 0) ? turn_duration_left_ : turn_duration_right_;
            use_dynamic_turn_from_sign_ = false;
            ROS_WARN("No sign center collected on timeout; fallback to base turn params: angle=%.2f, duration=%.2f",
                     turn_angle_from_sign_, turn_duration_from_sign_);
        } else {
            ComputeTurnFromSignCenter(judged_direction, cx, turn_angle_from_sign_, turn_duration_from_sign_, turn_scale_from_sign_);
            use_dynamic_turn_from_sign_ = true;
            ROS_WARN("Dynamic turn from sign (final): dir=%s, cx=%.1f => scale=%.3f, angle=%.2f, duration=%.2f",
                     judged_direction > 0 ? "LEFT" : "RIGHT", cx, turn_scale_from_sign_, turn_angle_from_sign_, turn_duration_from_sign_);
        }
    }
}

void RacingControlFinal::ComputeTurnFromSignCenter(int dir, float sign_center_x, float& out_angle, float& out_duration, float& out_scale) const {
    // 相机宽度按 640 处理：由于摄像头是歪的，实际中心点为 350。
    // 左侧范围 0-350（宽度350），右侧范围 350-640（宽度290）
    constexpr float kImgCenterX = 350.0f;
    constexpr float kLeftWidth = 350.0f;   // 左侧宽度
    constexpr float kRightWidth = 290.0f;  // 右侧宽度

    const float base_angle = (dir > 0) ? turn_angle_left_ : turn_angle_right_;
    const float base_duration = (dir > 0) ? turn_duration_left_ : turn_duration_right_;

    // 归一化偏移：[-1, 1]，右侧为正。根据指示牌位置使用不同的归一化宽度
    float offset_norm;
    if (sign_center_x >= kImgCenterX) {
        // 框在右侧，使用右侧宽度归一化
        offset_norm = (sign_center_x - kImgCenterX) / kRightWidth;
    } else {
        // 框在左侧，使用左侧宽度归一化
        offset_norm = (sign_center_x - kImgCenterX) / kLeftWidth;
    }
    offset_norm = std::max(-1.0f, std::min(1.0f, offset_norm));

    // 右转：框越靠右(偏移越大) => angle/time 越大
    // 左转：框越靠左(偏移越小) => angle/time 越大（与右转相反）
    const bool is_right = (dir < 0);
    const float signed_offset = is_right ? offset_norm : -offset_norm;
    float scale = 1.0f + turn_center_coeff_ * signed_offset;

    // 防止出现负值（只要系数 <= 1.0 且 offset_norm 在 [-1,1]，理论上不会为负）
    scale = std::max(0.05f, scale);

    // 不再对角度做动态缩放，仅对时长做缩放
    out_angle = base_angle;
    out_duration = base_duration * scale;
    out_scale = scale;
    if (dir > 0) {
          ROS_INFO("Computed LEFT turn from sign center: offset_norm=%.3f, scale=%.3f, angle=%.2f, duration=%.2f",
                   offset_norm, scale, out_angle, out_duration);  
    }else {
          ROS_INFO("Computed RIGHT turn from sign center: offset_norm=%.3f, scale=%.3f, angle=%.2f, duration=%.2f",
                   offset_norm, scale, out_angle, out_duration);  
    }
}

void RacingControlFinal::CheckZebraDisappear() {
    if (zebra_detected_.load() == 1) {
        double time_since_last_zebra = (ros::Time::now() - last_zebra_time_).toSec();
        if (time_since_last_zebra > zebra_disappear_timeout_) {
            zebra_detected_.store(2);
            ROS_INFO("Zebra disappeared! Close enough, preparing to stop...");
        }
    }
}

int RacingControlFinal::JudgeDirection() const {
    int sum = 0;
    for (int i = 0; i < direction_count_; i++) {
        sum += direction_history_[i];
    }
    
    ROS_INFO("Direction sum: %d (left=%d, right=%d)", 
             sum, (direction_count_ + sum) / 2, (direction_count_ - sum) / 2);
    
    if (sum > 0) return 1;
    else if (sum < 0) return -1;
    else return default_direction_ > 0 ? 1 : -1;
}

// ==================== Session 5: 转向 + y_obs检测 ====================

void RacingControlFinal::ExecuteSession5() {
    // 如果已完成，直接返回
    if (session5_complete_) {
        return;
    }
    
    turn_count_++;
    int current_direction = original_direction_;
    
    // 第一次：按原始方向转弯避障
    // 第二次：反向转弯回到原车道（超车逻辑）
    if (turn_count_ == 1) {
        ROS_INFO("=== Turn #1: Avoiding obstacle, direction=%s ===", 
                 current_direction > 0 ? "LEFT" : "RIGHT");
    } else if (turn_count_ == 2) {
        current_direction = -original_direction_;
        ROS_INFO("=== Turn #2: Returning to lane (overtaking), direction=%s ===", 
                 current_direction > 0 ? "LEFT" : "RIGHT");
    }
    
    // 设置当前转向方向（用于 ExecuteTurnAndCorrect）
    direction_.store(current_direction);
    
    // 执行转向+回正
    ExecuteTurnAndCorrect();
    
    ROS_INFO("Turn/Correct finished, now processing line following and y_obs detection...");
    
    // 第二次转向完成后，持续巡线
    if (turn_count_ >= 2) {
        session5_complete_ = true;
        return;
    }
    
    // 检查是否需要重复（只在第一次转向后检查）
    bool need_repeat = WaitForYObsDisappear();
    
    if (!need_repeat) {
        session5_complete_ = true;
    }
    // 如果 need_repeat = true，下次 Execute() 会再次进入 ExecuteSession5()
}

void RacingControlFinal::ExecuteTurnAndCorrect() {
    int dir = direction_.load();
    float isLeft = (dir > 0) ? 1.0f : -1.0f;
    float turn_angle, turn_duration, correct_duration;
    
    // 第一次变道：使用第一次变道参数
    if (turn_count_ == 1) {
        turn_angle = (dir > 0) ? turn_angle_left_ : turn_angle_right_;
        turn_duration = (dir > 0) ? turn_duration_left_ : turn_duration_right_;
        correct_duration = (dir > 0) ? correct_duration_left_ : correct_duration_right_;
    }
    // 第二次变道：使用第二次变道参数
    else {
        turn_angle = (dir > 0) ? turn2_angle_left_ : turn2_angle_right_;
        turn_duration = (dir > 0) ? turn2_duration_left_ : turn2_duration_right_;
        correct_duration = (dir > 0) ? correct2_duration_left_ : correct2_duration_right_;
    }

    // 第二次转向（回到车道）：速度降为 0.5
    const float speed_scale = (turn_count_ == 2) ? 0.5f : 1.0f;
    const float cmd_speed = lineSpeed_ * speed_scale;

    // 仅对“指示牌决定的第一次转向”启用动态角度/时间
    if (enable_dynamic_turn_from_sign_ && use_dynamic_turn_from_sign_ && turn_count_ == 1 && dir == original_direction_) {
        turn_angle = turn_angle_from_sign_;
        turn_duration = turn_duration_from_sign_;
        // 回正时长也按同一比例缩放，保证转向/回正节奏一致
        correct_duration *= turn_scale_from_sign_;
        ROS_WARN("Using dynamic turn params: scale=%.3f, angle=%.2f, duration=%.2f, correct=%.2f",
                 turn_scale_from_sign_, turn_angle, turn_duration, correct_duration);
    }

    const float effective_turn_angle = turn_angle;
    
    // ========== 阶段1: 转向 ==========
    ROS_INFO("Phase 1: Turning %s for %.1f seconds...", dir > 0 ? "LEFT" : "RIGHT", turn_duration);
    ros::Time start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < turn_duration) {
        float angle = isLeft * effective_turn_angle;
        ROS_INFO_THROTTLE(1, "Turning: angle=%.1f", angle);
        PublishCommand(cmd_speed, angle);
        ros::Duration(0.1).sleep();
    }
    
    // ========== 阶段2: 回正 ==========
    ROS_INFO("Phase 2: Correcting for %.1f seconds...", correct_duration);
    start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < correct_duration) {
        float angle = -0.8f * isLeft * effective_turn_angle;
        ROS_INFO_THROTTLE(1, "Correcting: angle=%.1f", angle);
        PublishCommand(cmd_speed, angle);
        ros::Duration(0.1).sleep();
    }
    
    ROS_INFO("Turn and correct cycle completed!");
}

bool RacingControlFinal::WaitForYObsDisappear() {
    ROS_INFO("Starting y_obs detection phase, current y_obs_flag=%d", y_obs_flag_);

    uint64_t last_counted_seq = 0;
    int miss_frames = 0;
    
        // 简化逻辑：根据当前转向方向直接设置偏移
        static float y_obs_offset_ = 40.0f; // 可在构造函数中读取参数
        static float default_center_ = 320.0f;
        if (y_obs_detected_) {
            int dir = direction_.load(); // >0左转，<0右转
            float offset = (dir > 0) ? -y_obs_offset_ : y_obs_offset_;
            adjusted_center_ = default_center_ + offset;
            ROS_INFO_THROTTLE(1.0, "Y_OBS detected, direction=%d, set adjusted_center_=%.1f", dir, adjusted_center_);
        } else {
            adjusted_center_ = default_center_;
        }
    while (ros::ok()) {
        ros::spinOnce();

        // flag=2 且当前未检测到 y_obs 的“等待确认消失”过程：需要降速
        if (y_obs_flag_ == 2 && !y_obs_detected_) {
            // 这里的降速参数与 ExecuteTurnAndCorrect 中保持一致：第二次转向(回到车道)为 0.5，否则为 1.0。
            // 等待确认消失阶段对应“下一次即将进行的转向”，因此用 turn_count_+1 来计算。
            const int upcoming_turn_count = turn_count_ + 1;
            const float speed_scale = (upcoming_turn_count == 2) ? 0.0f : 1.0f;
            const float add_speed = lineSpeed_ * (speed_scale - 1.0f);
            ProcessLineAndObstacle(add_speed);
        } else {
            ProcessLineAndObstacle(0.0f);
        }
        
        if (y_obs_detected_) {
            ROS_INFO_THROTTLE(0.2, "Y_OBS present in frame...");
        }
        
        const uint64_t current_seq = session5_last_target_seq_.load(std::memory_order_relaxed);
        const bool has_new_frame = (current_seq != 0 && current_seq != last_counted_seq);

        if (y_obs_flag_ == 1) {
            if (!y_obs_detected_) {
                ROS_INFO("Y_OBS disappeared! Changing flag from 1 to 2, waiting %d consecutive frames...", y_obs_wait_timeout_);
                y_obs_flag_ = 2;
                // 丢失的当前帧也计入连续未检测帧
                miss_frames = 1;
                const uint64_t current_seq = session5_last_target_seq_.load(std::memory_order_relaxed);
                if (current_seq != 0) {
                    last_counted_seq = current_seq;
                }
            }
        } else if (y_obs_flag_ == 2) {
            if (y_obs_detected_) {
                ROS_INFO("Y_OBS reappeared! Changing flag back from 2 to 1");
                y_obs_flag_ = 1;
                miss_frames = 0;
            } else if (has_new_frame) {
                last_counted_seq = current_seq;
                miss_frames++;

                if (miss_frames >= y_obs_wait_timeout_) {
                    ROS_WARN("Y_OBS confirmed disappeared for %d consecutive frames, repeating turn maneuver...", y_obs_wait_timeout_);
                    return true;
                }

                ROS_INFO_THROTTLE(0.2, "Waiting for y_obs disappear confirmation: %d/%d frames", miss_frames, y_obs_wait_timeout_);
            }
        } else if (y_obs_flag_ == 0) {
            ROS_INFO_THROTTLE(1.0, "Waiting for y_obs detection (flag=0)...");
        }
        
        ros::Duration(0.05).sleep();
    }
    
    return false;
}

void RacingControlFinal::ProcessLineAndObstacle(float add_speed) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        if (!point_msg->lane_points.empty()) {
            LineFollowing(point_msg->lane_points[0], add_speed);
        }
        lock.lock();
        point_queue_.pop();
    }
}

// ==================== 辅助函数 ====================

void RacingControlFinal::UpdateAdjustedCenter(const obstacle_detector::PerceptionTarget& target) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        
        if (!point_msg->lane_points.empty() &&
            !point_msg->lane_points[0].points.empty() &&
            point_msg->lane_points[0].points[0].point.size() >= 3) {

            float left = point_msg->lane_points[0].points[0].point[1].x;
            float right = point_msg->lane_points[0].points[0].point[2].x;
            float obstacle_center = (target.x1 + target.x2) / 2.0f;

            //若同时检测到多个蓝色锥桶，则选择在赛道内的一个进行避障，即obstacle_center在left和right之间
            if(obstacle_center < left || obstacle_center > right){
                ROS_WARN("Obstacle center %.1f out of lane bounds (left=%.1f, right=%.1f), skipping adjustment", obstacle_center, left, right);
                return;
            }

            float left_dist = obstacle_center - left;
            float right_dist = right - obstacle_center;

            ROS_INFO("Lane: left=%.1f, right=%.1f, obstacle_center=%.1f", left, right, obstacle_center);

            float target_offset = 0.0;
            if (left_dist < right_dist) {
                target_offset = -37.0;
                judgeLeft_ = false;
                ROS_WARN("Obstacle on LEFT, shifting target RIGHT (offset=%.1f)", target_offset);
            } else {
                target_offset = 37.0;
                judgeLeft_ = true;
                ROS_WARN("Obstacle on RIGHT, shifting target LEFT (offset=%.1f)", target_offset);
            }

            adjusted_center_ = 320.0 + target_offset;
            ROS_INFO("Updated adjusted_center_ to %.1f", adjusted_center_);
        }
    }
}

void RacingControlFinal::LineFollowing(const obstacle_detector::Target& point_msg, float add_speed) {
    ros_serial::to32 to32_msg;
    float angle = 0.0;

    ROS_INFO("LineFollowing...");

    // ========== 获取赛道数据 ==========
    if (point_msg.points.empty() || point_msg.points[0].point.size() < 3) {
        ROS_WARN("Invalid point data, skipping frame");
        return;
    }

    float left = point_msg.points[0].point[1].x;
    float right = point_msg.points[0].point[2].x;
    float mid = point_msg.points[0].point[0].x;

    ROS_INFO("left_x:%.1f, mid_x:%.1f, right_x:%.1f", left, mid, right);

    // ========== 检查是否正在执行恢复动作 ==========
    if (is_recovering_) {
        double elapsed = (ros::Time::now() - recovery_start_time_).toSec();
        
        // 检查赛道是否已经恢复正常
        if (right - left >= 155 && right - left <= 215) {
            // 赛道宽度恢复正常，立即停止恢复
            is_recovering_ = false;
            abnormal_lane_count_ = 0;
            ROS_INFO("========== RECOVERY SUCCESS: Lane found! ==========");
            ROS_INFO("Lane width: %.1f, stopped recovery at %.2fs/%.1fs", 
                     right - left, elapsed, recovery_duration_);
            // 不return，继续执行正常巡线逻辑
        } else if (elapsed < recovery_duration_) {
            // 赛道仍未恢复，继续执行恢复动作
            to32_msg.speed = lineSpeed_ * 0.7;
            to32_msg.angle = recovery_angle_;
            to32_msg.run_flag = true;
            publisher_.publish(to32_msg);
            
            ROS_WARN_THROTTLE(0.5, "RECOVERING: angle=%.1f (opposite of %.1f), elapsed=%.2fs/%.1fs", 
                            recovery_angle_, -recovery_angle_, elapsed, recovery_duration_);
            return;
        } else {
            // 恢复时间到，重置状态
            is_recovering_ = false;
            abnormal_lane_count_ = 0;
            ROS_INFO("Recovery timeout! Returning to normal line following.");
        }
    }

    // ========== 检测赛道宽度异常 ==========
    if (right - left < 155 || right - left > 215) {
        abnormal_lane_count_++;
        ROS_WARN("Lane width abnormal (%.1f), count: %d/%d", 
                 right - left, abnormal_lane_count_, abnormal_threshold_);
        
        // 达到阈值，触发恢复机制
        if (abnormal_lane_count_ >= abnormal_threshold_) {
            is_recovering_ = true;
            recovery_start_time_ = ros::Time::now();
            
            // 角度取反：大小相同，方向相反
            recovery_angle_ = -last_angle_;
            
            ROS_WARN("========== TRIGGERING RECOVERY MECHANISM ==========");
            ROS_WARN("Last angle: %.1f, Recovery angle: %.1f (opposite direction)", 
                     last_angle_, recovery_angle_);
            ROS_WARN("Will turn opposite direction for %.1fs", recovery_duration_);
            
            abnormal_lane_count_ = 0;  // 重置计数
            return;
        }
        
        // 未达到阈值，继续使用原有逻辑
        angle = last_angle_ * 0.8;
        angle = std::max(-10.0f, std::min(10.0f, angle));
        
        to32_msg.speed = lineSpeed_ * 0.7;
        to32_msg.angle = angle;
        to32_msg.run_flag = true;
        publisher_.publish(to32_msg);
        return;
    }
    
    // ========== 赛道宽度正常，重置异常计数 ==========
    if (abnormal_lane_count_ > 0) {
        ROS_INFO("Lane width returned to normal, resetting abnormal count from %d to 0", 
                 abnormal_lane_count_);
    }
    abnormal_lane_count_ = 0;

    // ========== 正常的巡线控制 ==========
    float error = mid - adjusted_center_;

    if (isAvoiding_) {
        ROS_INFO("AVOIDING: mid=%.1f, target=%.1f, error=%.1f", mid, adjusted_center_, error);
    } else {
        ROS_INFO("NORMAL: mid=%.1f, error=%.1f", mid, error);
    }

    float filtered_error = ApplyMovingAverage(error_buffer_, error);
    ROS_INFO("error: raw=%.1f, filtered=%.1f", error, filtered_error);

    angle = -1.0 * follow_p_ * filtered_error;
    angle = std::max(-10.0f, std::min(10.0f, angle));

    if (!IsAngleValid(angle)) {
        ROS_WARN("Angle change too large (current=%.1f, last=%.1f), skipping frame", angle, last_angle_);
        return;
    }

    angle = ApplyMovingAverage(angle_buffer_, angle);
    last_angle_ = angle;
    first_frame_ = false;

    ROS_INFO("target_angle: %.1f", angle);

    // 避障时仅降速，转向角改为原来的1.4倍
    float speed_scale = isAvoiding_ ? 0.8f : 1.0f;
    float angle_scale = isAvoiding_ ? 1.4f : 1.0f;
    to32_msg.speed = (lineSpeed_ + add_speed) * speed_scale;
    to32_msg.angle = angle * angle_scale;
    to32_msg.run_flag = true;
    publisher_.publish(to32_msg);
}

void RacingControlFinal::PublishCommand(float speed, float angle) {
    ros_serial::to32 msg;
    msg.speed = speed;
    msg.angle = angle;
    msg.run_flag = true;
    publisher_.publish(msg);
}

void RacingControlFinal::PublishStop() {
    ros_serial::to32 msg;
    msg.speed = 0.0;
    msg.angle = 0.0;
    msg.run_flag = false;
    publisher_.publish(msg);
}

float RacingControlFinal::ApplyMovingAverage(std::deque<float>& buffer, float new_value) {
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

bool RacingControlFinal::IsAngleValid(float current_angle) {
    if (first_frame_) {
        return true;
    }

    float diff = std::abs(current_angle - last_angle_);
    return diff <= max_angle_diff_;
}

// ==================== 测试用 main 函数 ====================

int main(int argc, char** argv) {
    ros::init(argc, argv, "racing_control_final");

    RacingControlFinal controller;
    ros::Rate rate(30);

    ROS_INFO("========================================");
    ROS_INFO("  Racing Control Final - Integrated    ");
    ROS_INFO("========================================");
    ROS_INFO("State Machine:");
    ROS_INFO("  SESSION_2: Line following + Obstacle avoidance + Speed bump");
    ROS_INFO("             -> Transition: 3 consecutive zebra frames");
    ROS_INFO("  SESSION_4: Zebra stop + Wait 11s + Collect direction");
    ROS_INFO("             -> Transition: 11s elapsed AND direction received");
    ROS_INFO("  SESSION_5: Turn + Correct + y_obs detection");
    ROS_INFO("             -> Transition: All turns complete");
    ROS_INFO("  SESSION_COMPLETE: Continue line following");
    ROS_INFO("========================================");
    ROS_INFO("Press Ctrl+C to stop");

    while (ros::ok()) {
        ros::spinOnce();
        controller.Execute();

        // 打印当前状态
        SessionState state = controller.GetState();
        const char* state_str = "";
        switch (state) {
            case SessionState::SESSION_2: state_str = "SESSION_2"; break;
            case SessionState::SESSION_4: state_str = "SESSION_4"; break;
            case SessionState::SESSION_5: state_str = "SESSION_5"; break;
            case SessionState::SESSION_COMPLETE: state_str = "SESSION_COMPLETE"; break;
        }
        ROS_INFO_THROTTLE(5, "Current state: %s", state_str);

        rate.sleep();
    }

    ROS_INFO("Racing Control Final stopped.");
    return 0;
}
