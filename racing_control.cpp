#include "racing_control.h"
#include <std_msgs/String.h>
#include <atomic>


RacingControlNode::RacingControlNode(const std::string& node_name) {
    ros::NodeHandle nh;
    if (!msg_process_) {
        msg_process_ = std::make_shared<std::thread>(&RacingControlNode::MessageProcess, this);
    }
    point_subscriber_ = nh.subscribe("/line_detector/center", 10, &RacingControlNode::subscription_callback_point, this);
    target_subscriber_ = nh.subscribe("/obstacle/detection", 10, &RacingControlNode::subscription_callback_target, this);
    direction_subscriber_ = nh.subscribe("/yolo_fastestv2/direction", 1, &RacingControlNode::direction_callback, this);
    publisher_ = nh.advertise<ros_serial::to32>("/cmd_vel", 5);

    // 初始化方向消息
    latest_direction_.store(0);

    nh.getParam("avoidBottom", avoidBottom);
    nh.getParam("lineSpeed", lineSpeed);
    nh.getParam("LeftOrRight", LeftOrRight);
    nh.getParam("initial_wait_time", initial_wait_time_);

    ROS_INFO("RacingControlNode initialized!");
    ROS_INFO("Parameters: avoidBottom=%d, lineSpeed=%.2f, initial_wait=%.1fs", 
             avoidBottom, lineSpeed, initial_wait_time_);
}


RacingControlNode::~RacingControlNode() {
    if (msg_process_ && msg_process_->joinable()) {
        process_stop_ = true;
        msg_process_->join();
        msg_process_ = nullptr;
    }
}

// 方向消息回调
void RacingControlNode::direction_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "left") {
        latest_direction_.store(1);
    } else if (msg->data == "right") {
        latest_direction_.store(-1);
    }
}

// 巡线点消息回调
void RacingControlNode::subscription_callback_point(const obstacle_detector::PerceptionTargets::ConstPtr& point_msg) {
    // 记录当前帧左右车道线的X坐标，用于后续避障判断
    left_x[point_msg->image_seq] = point_msg->targets[0].points[0].point[1].x;
    right_x[point_msg->image_seq] = point_msg->targets[0].points[0].point[2].x;
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    point_queue_.push(point_msg);
    if (point_queue_.size() > 1) {
      point_queue_.pop();
    }
}

// 障碍物目标消息回调
void RacingControlNode::subscription_callback_target(const obstacle_detector::PerceptionTargets::ConstPtr& targets_msg) {
    sub_target_ = true;
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    targets_queue_.push(targets_msg);
    if (targets_queue_.size() > 1) {
      targets_queue_.pop();
    }
}

// 发布停止指令
void RacingControlNode::PublishStop() {
    ros_serial::to32 to32_msg;
    to32_msg.speed = 0.0;
    to32_msg.angle = 0.0;
    to32_msg.run_flag = false;
    publisher_.publish(to32_msg);
}

// Session 1: mode 0 - 原地静止30秒
void RacingControlNode::HandleModeStop() {
    ROS_INFO_THROTTLE(5, "Mode 0: Standing still...");
    PublishStop();
}

// Session 2: mode 1 - 只参与巡线
void RacingControlNode::HandleModeLineFollow() {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        LineFollowing(point_msg->targets[0]);
        lock.lock();
        point_queue_.pop();
    }
}

// Session 2: mode 2 - 巡线 + 避障
void RacingControlNode::HandleModeLineAvoid() {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    
    // 检查是否有障碍物需要避开
    if (!targets_queue_.empty() && sub_target_) {
        auto target_msg = targets_queue_.top();
        lock.unlock();
        
        // 判断是否需要避障
        if (!target_msg->targets.empty()) {
            auto& target = target_msg->targets[0];
            // 如果障碍物在避障区域内，执行避障
            if (target.y2 >= avoidBottom) {
                ObstaclesAvoiding(target);
                lock.lock();
                targets_queue_.pop();
                return;
            }
        }
        lock.lock();
        targets_queue_.pop();
    }
    
    // 没有障碍物或障碍物较远，执行巡线
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        LineFollowing(point_msg->targets[0]);
        lock.lock();
        point_queue_.pop();
    }
}

// Session 3: mode 3 - 巡线 + 过减速带
void RacingControlNode::HandleModeSpeedBump() {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (!point_queue_.empty()) {
        auto point_msg = point_queue_.top();
        lock.unlock();
        // 过减速带时降低速度
        LineFollowing(point_msg->targets[0], -0.5);
        lock.lock();
        point_queue_.pop();
    }
}

// Session 4 & 5: mode 4 - 停车等待方向 + 转向
void RacingControlNode::HandleModeTurn() {
    ROS_INFO("Session 4: Stopping at zebra crossing, waiting for direction...");

    ros::Rate rate(10);
    ros::Time start_time = ros::Time::now();
    int direction = 0;

    // 等待方向消息（最多11秒）
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 11.0) {
        ros::spinOnce();
        PublishStop();

        int latest = latest_direction_.load();
        if (latest != 0) {
            direction = latest;
            break;
        }
        rate.sleep();
    }

    // 未收到消息，使用默认参数
    if (direction == 0) {
        direction = (LeftOrRight > 0) ? 1 : -1;
        ROS_WARN("No direction message received, fallback to param LeftOrRight: %d", direction);
    } else {
        ROS_INFO("Direction message received: %s", direction == 1 ? "left" : "right");
    }

    LeftOrRight = direction;
    
    ROS_INFO("Session 5: Starting turn maneuver...");
    BlessOurTurn();

    // 转向完成后清空状态
    latest_direction_.store(0);

    // 转向完成，切换到巡线模式
    current_mode_ = RacingMode::LINE_FOLLOW;
    current_session_ = 6; // 标记已完成所有session
    ROS_INFO("Turn complete, switching to line following mode.");
}

// 切换到下一个 session
void RacingControlNode::SwitchToNextSession() {
    current_session_++;
    ROS_INFO("Switching to session %d", current_session_);

    switch (current_session_) {
        case 2:
            current_mode_ = RacingMode::LINE_AVOID;
            ROS_INFO("Session 2: Line following + Obstacle avoidance");
            break;
        case 3:
            current_mode_ = RacingMode::LINE_SPEED_BUMP;
            ROS_INFO("Session 3: Line following + Speed bump");
            break;
        case 4:
            current_mode_ = RacingMode::TURN;
            ROS_INFO("Session 4: Stop at zebra crossing");
            break;
        default:
            current_mode_ = RacingMode::LINE_FOLLOW;
            ROS_INFO("Default: Line following");
            break;
    }
}

void RacingControlNode::MessageProcess() {
    ros::Rate loop_rate(50); // 50Hz 主循环

    // 初始化比赛开始时间
    race_start_time_ = ros::Time::now();
    current_session_ = 1;
    current_mode_ = RacingMode::STOP;

    ROS_INFO("Race started! Session 1: Standing still for %.1f seconds", initial_wait_time_);

    while (!process_stop_ && ros::ok()) {
        ros::spinOnce();

        double elapsed = (ros::Time::now() - race_start_time_).toSec();

        // Session 1: 前30秒静止
        if (current_session_ == 1) {
            if (elapsed < initial_wait_time_) {
                HandleModeStop();
                ROS_INFO_THROTTLE(5, "Session 1: %.1f seconds remaining", initial_wait_time_ - elapsed);
            } else {
                SwitchToNextSession(); // 进入 session 2
            }
            loop_rate.sleep();
            continue;
        }

        // 根据当前模式执行对应操作
        switch (current_mode_) {
            case RacingMode::STOP:
                HandleModeStop();
                break;

            case RacingMode::LINE_FOLLOW:
                HandleModeLineFollow();
                break;

            case RacingMode::LINE_AVOID:
                HandleModeLineAvoid();
                // TODO: 检测是否进入减速带区域，若是则切换到 session 3
                // if (detected_speed_bump) SwitchToNextSession();
                break;

            case RacingMode::LINE_SPEED_BUMP:
                HandleModeSpeedBump();
                // TODO: 检测是否到达斑马线，若是则切换到 session 4
                // if (detected_zebra_crossing) SwitchToNextSession();
                break;

            case RacingMode::TURN:
                HandleModeTurn();
                break;
        }

        loop_rate.sleep();
    }
}

// 巡线核心子函数
void RacingControlNode::LineFollowing(const obstacle_detector::Target &point_msg, float add_speed) {
    
    ros_serial::to32 to32_msg;
    
    float angle = 0.0;

    // 优先处理修正角度队列（通常用于避障后的回正）
    if (!correctAngle.empty()) {
        iscorrecting = true;
        angle = -0.8 * correctAngle.front();
        correctAngle.pop();
        ROS_WARN("CorrectAngle...%.1f", angle);
        if(correctAngle.empty()) iscorrecting = false;
    }
    else {
        // 正常巡线逻辑
        ROS_WARN("LineFollowing...");
        ROS_INFO("left_x:%.1f, mid_x:%.1f, right_x:%.1f", point_msg.points[0].point[1].x, point_msg.points[0].point[0].x, 
                                                          point_msg.points[0].point[2].x);

        // 检查车道线宽度是否异常
        if(point_msg.points[0].point[2].x - point_msg.points[0].point[1].x < 100 ||
        point_msg.points[0].point[2].x - point_msg.points[0].point[1].x > 200) {
            ROS_WARN("the line too short or too long");
            return;
        }
        
        float x = point_msg.points[0].point[0].x;
        
        // 计算偏差（假设图像中心为320.0）                                              
        float error = x - 320.0;

        // P控制器计算转向角
        angle = -1.0 * follow_p * error;

        // 限制角度范围
        if(angle < -10.0) angle = -10.0;
        if(angle > 10.0) angle = 10.0;

        ROS_INFO("target_angle:%.1f", angle);
    }

    // 设置速度和角度
    to32_msg.speed = lineSpeed + add_speed;
    to32_msg.angle = angle;
    to32_msg.run_flag = true;
  

    publisher_.publish(to32_msg);
}


// 计算避障时的目标路径点X坐标
float RacingControlNode::FindAvoidTargetPoint(const int a[], int current_l, int current_r) {
    // 计算当前障碍物宽度与记录的障碍物宽度的比例
    float ratio = 1.0 * (current_r - current_l) / (a[3] - a[2]);
    // 根据比例推算新的左右边界
    float left_new = current_l - ratio * (std::max(0, a[2] -a[1]));
    float right_new = ratio * (std::max(0, a[4] - a[3])) + current_r;
    ROS_INFO("left_new:%.1f  right_new:%.1f", left_new, right_new);
    
    float mid_new = (left_new + right_new) / 2;
    // 根据避障方向（左/右）选择目标点
    float target_x = judgeLeft ? (mid_new - (right_new - left_new) / 4) : (mid_new + (right_new - left_new) / 4);
    
    float mid_obstacle = (current_l + current_r) / 2;
    // 限制目标点，确保不会撞上障碍物
    target_x = target_x <= mid_obstacle ? (std::min(target_x, std::max((float)0.0, mid_obstacle - 80))) : (std::max(target_x, mid_obstacle + 55));
    return target_x;
}

// 避障控制函数
void RacingControlNode::ObstaclesAvoiding(const obstacle_detector::PerceptionTarget& target, float add_speed) {
    ROS_ERROR("ObstaclesAvoiding...");

    ros_serial::to32 to32_msg;
    
    // 如果没有有效的原始点记录，直接返回
    if(origin_points[2] == origin_points[3]) return ;

    // 计算避障目标点
    float target_point = FindAvoidTargetPoint(origin_points, target.x1,target.x2);
                                            
    ROS_INFO("target_point:%.1f", target_point);
    float error = target_point - 320.0; // 目标点偏差

    // P控制器计算避障转向角
    float angle = -1.0 * avoid_p * error;
    
    ROS_INFO_STREAM("TURN LEFT: " << (judgeLeft ? "true" : "false"));
    
    // 记录回正角度，用于避障后的路径恢复
    if(judgeLeft) {
        if(correctAngle.size() < 6) {
            correctAngle.push(angle / (1.0 + 0.90 * correctAngle.size()));
        }
    }
    else {
        if(correctAngle.size() < 10) { 
            correctAngle.push(-1.25 * (10.0 - angle) / (1.0 + 0.04 * correctAngle.size()));
        }
    }
    
    ROS_INFO("obstacle_angle:%.1f", angle);

    to32_msg.speed = lineSpeed + add_speed;
    to32_msg.angle = angle;
    to32_msg.run_flag = true;

    publisher_.publish(to32_msg);
}

void RacingControlNode::BlessOurTurn() {
    float isLeft = LeftOrRight > 0 ? 1.0 : -1.0;
    float angle = 10.0;
    
    ROS_INFO("Turn direction: %s", LeftOrRight > 0 ? "left" : "right");
    
    // 阶段1: 转向 (5.8秒)
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.8) {
        ros::spinOnce();
        
        ros_serial::to32 to32_msg;
        to32_msg.speed = lineSpeed;
        to32_msg.angle = isLeft * angle;
        to32_msg.run_flag = true;
        publisher_.publish(to32_msg);

        ros::Duration(0.1).sleep();
    }
    
    // 阶段2: 回正 (2.6秒)
    start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 2.6) {
        ros::spinOnce();

        ros_serial::to32 to32_msg;
        to32_msg.speed = lineSpeed;
        to32_msg.angle = -0.8 * isLeft * angle;
        to32_msg.run_flag = true;
        publisher_.publish(to32_msg);

        ros::Duration(0.1).sleep();
    }
    
    ROS_INFO("Turn complete!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "racing_control");
    RacingControlNode racing_control_node("RacingControlNode");
    ros::spin();
    return 0;
}
