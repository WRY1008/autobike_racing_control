#ifndef YOLO_FASTESTV2_H_
#define YOLO_FASTESTV2_H_

// NCNN 框架头文件 
#include "net.h"

#include <vector>
#include <opencv2/opencv.hpp>

// ROS 头文件
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

// 目标检测框消息头文件
#include "obstacle_detector/PerceptionTargets.h"
#include "obstacle_detector/Copyimage.h"


class TargetBox
{
private:
    float getWidth() { return (x2 - x1); };
    float getHeight() { return (y2 - y1); };

public:
    int x1;
    int y1;
    int x2;
    int y2;

    int cate;
    float score;

    float area() { return getWidth() * getHeight(); };
};

class yoloFastestv2
{
private:
    ncnn::Net net;
    std::vector<float> anchor;

    const char* inputName;    // 改为 const char*
    const char* outputName1;  // 改为 const char*
    const char* outputName2;  // 改为 const char*

    int numAnchor;
    int numOutput;
    int numThreads;
    int numCategory;
    int inputWidth, inputHeight;

    float nmsThresh;

    int nmsHandle(std::vector<TargetBox> &tmpBoxes, std::vector<TargetBox> &dstBoxes);
    int getCategory(const float *values, int index, int &category, float &score);
    int predHandle(const ncnn::Mat *out, std::vector<TargetBox> &dstBoxes, 
                   const float scaleW, const float scaleH, const float thresh);


public:
    yoloFastestv2();
    ~yoloFastestv2();

    int loadModel(const char* paramPath, const char* binPath);
    int detection(const cv::Mat srcImg, std::vector<TargetBox> &dstBoxes, 
                  const float thresh = 0.3);
};

// yolo_obstacle_node.h
class YoloObstacleNode {
private:
    ros::Subscriber img_sub_;
    ros::Publisher img_pub_;
    ros::Publisher msg_pub_;
    ros::Publisher direction_pub_;  // 方向发布者
    ros::Publisher zebra_pub_;  // 斑马线发布者
    
    
    yoloFastestv2 detector_;
    
    // 帧率计算
    ros::Time last_time_;
    int frame_count_;
    double fps_;
    
    // 类别名称数组
    std::vector<std::string> class_names_;
    
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

public:
    YoloObstacleNode(ros::NodeHandle& nh, 
                     const std::string& param_path,
                     const std::string& bin_path);
    ~YoloObstacleNode();

};

#endif // YOLO_FASTESTV2_H_