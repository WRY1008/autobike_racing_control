#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <vector>
#include "obstacle_detector/PerceptionTargets.h"
#include "obstacle_detector/Copyimage.h"

using namespace cv;
using namespace std;
using namespace Ort;

// 定义检测框信息结构体
typedef struct BoxInfo {
    float x1, y1, x2, y2;  // 检测框的左上角和右下角坐标
    float score;           // 检测框的置信度得分
    int label;             // 检测框的类别标签
} BoxInfo;

// 定义 NanoDetPlusNode 类，用于目标检测
class NanoDetPlusNode {
public:
    // 构造函数，初始化模型路径、类别文件和检测阈值
    NanoDetPlusNode(ros::NodeHandle& nh, const std::string& model_path, const std::string& classesFile, float nms_threshold, float objThreshold);
    void imageCallback(const obstacle_detector::Copyimage::ConstPtr& msg);  // ROS 图像回调函数

private:
    void detect(Mat& cv_image, const sensor_msgs::ImageConstPtr& msg);  // 检测函数
    Mat resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left);  // 图像缩放函数
    void normalize_(Mat img);  // 图像归一化处理函数
    void softmax_(const float* x, float* y, int length);  // Softmax 函数
    void generate_proposal(vector<BoxInfo>& generate_boxes, const float* preds);  // 生成候选框
    void nms(vector<BoxInfo>& input_boxes);  // 非极大值抑制

    ros::Subscriber image_sub_;  // 图像订阅器
    ros::Publisher image_pub_;  // 图像发布器
    ros::Publisher msg_publisher_; // 检测框发布器

    float score_threshold;  // 置信度阈值
    float nms_threshold;  // 非极大值抑制阈值
    vector<string> class_names;  // 类别名称列表
    int num_class;  // 类别数目
    vector<float> input_image_;  // 输入图像的归一化数据
    Env env = Env(ORT_LOGGING_LEVEL_ERROR, "nanodetplus");  // ONNX Runtime 环境
    Ort::Session* ort_session = nullptr;  // ONNX 模型会话
    SessionOptions sessionOptions = SessionOptions();  // 会话选项
    vector<char*> input_names;  // 模型输入名称
    vector<char*> output_names;  // 模型输出名称
    vector<vector<int64_t>> input_node_dims;  // 输入节点维度
    vector<vector<int64_t>> output_node_dims;  // 输出节点维度
    int inpWidth;  // 模型输入宽度
    int inpHeight;  // 模型输入高度
    int reg_max;  // 最大回归量
    const int num_stages = 4;
    const int stride[4] = {8, 16, 32, 64};  // 不同尺度的检测步长
    const float mean[3] = {103.53, 116.28, 123.675};  // 图像均值
    const float stds[3] = {57.375, 57.12, 58.395};  // 图像标准差
};

#endif // OBSTACLE_DETECTOR_H
