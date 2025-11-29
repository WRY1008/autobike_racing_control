#include <math.h>
#include <algorithm>
#include <deque>
#include "yolo-fastestv2.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

// ============ 前置声明：结构体定义（必须在使用前定义） ============

// 结构体：白色像素比例结果
struct WhitePixelRatio {
    double left_ratio;
    double right_ratio;
    std::string comparison;
};

// 结构体：保存白色像素比例历史记录
class RatioHistory {
public:
    static const int HISTORY_SIZE = 5;  // 历史记录长度
    std::deque<double> left_ratios;
    std::deque<double> right_ratios;
    
    bool isStable() {
        if (left_ratios.size() < HISTORY_SIZE || right_ratios.size() < HISTORY_SIZE) {
            return false;  // 历史数据不足
        }
        
        // 计算左右比例的方差
        double left_mean = 0, right_mean = 0;
        for (double val : left_ratios) left_mean += val;
        for (double val : right_ratios) right_mean += val;
        left_mean /= HISTORY_SIZE;
        right_mean /= HISTORY_SIZE;
        
        double left_variance = 0, right_variance = 0;
        for (double val : left_ratios) left_variance += (val - left_mean) * (val - left_mean);
        for (double val : right_ratios) right_variance += (val - right_mean) * (val - right_mean);
        left_variance /= HISTORY_SIZE;
        right_variance /= HISTORY_SIZE;
        
        // 判断稳定性：变异系数小于阈值
        const double STABILITY_THRESHOLD = 0.1;
        double left_cv = (left_mean > 0.001) ? sqrt(left_variance) / left_mean : 0;
        double right_cv = (right_mean > 0.001) ? sqrt(right_variance) / right_mean : 0;
        
        return (left_cv < STABILITY_THRESHOLD && right_cv < STABILITY_THRESHOLD);
    }

    void addRatio(double left, double right) {
        left_ratios.push_back(left);
        right_ratios.push_back(right);
        if (left_ratios.size() > HISTORY_SIZE) {
            left_ratios.pop_front();
            right_ratios.pop_front();
        }
    }
    
    void getAverageRatios(double& avg_left, double& avg_right) {
        avg_left = 0;
        avg_right = 0;
        for (double val : left_ratios) avg_left += val;
        for (double val : right_ratios) avg_right += val;
        if (!left_ratios.empty()) {
            avg_left /= left_ratios.size();
            avg_right /= right_ratios.size();
        }
    }
};

// 全局白色像素比例历史
static RatioHistory ratio_history;

// 前置声明：白色像素检测函数
WhitePixelRatio calculateWhiteRatioInDetectionBox(
    const cv::Mat& image, int x1, int y1, int x2, int y2);

// ============ yoloFastestv2 类实现 ============

//模型的参数配置
yoloFastestv2::yoloFastestv2()
{   
    printf("Creat yoloFastestv2 Detector...\n");
    //输出节点数
    numOutput = 2;
    //推理线程数
    numThreads = 1;
    //anchor num
    numAnchor = 3;
    //类别数目
    numCategory = 3;
    //NMS阈值
    nmsThresh = 0.2;

    //模型输入尺寸大小
    inputWidth = 352;
    inputHeight = 352;

    //模型输入输出节点名称
    inputName = "input.1";
    outputName1 = "770"; //22x22
    outputName2 = "772"; //11x11

    //打印初始化相关信息
    printf("numThreads:%d\n", numThreads);
    printf("inputWidth:%d inputHeight:%d\n", inputWidth, inputHeight);

    //anchor box w h (从 anchors6.txt)
    std::vector<float> bias {33.96,106.86, 36.90,31.16, 39.42,95.46, 44.36,43.98, 115.39,46.02, 115.87,33.57};

    anchor.assign(bias.begin(), bias.end());
}

yoloFastestv2::~yoloFastestv2()
{
    printf("Destroy yoloFastestv2 Detector...\n");
}

YoloObstacleNode::YoloObstacleNode(ros::NodeHandle& nh, 
                                   const std::string& param_path,
                                   const std::string& bin_path) {
    detector_.loadModel(param_path.c_str(), bin_path.c_str());

    // 加载模型并检查错误
    if (detector_.loadModel(param_path.c_str(), bin_path.c_str()) != 0) {
        ROS_ERROR("Failed to load YOLO model from:\n  param: %s\n  bin: %s", 
                  param_path.c_str(), bin_path.c_str());
        ros::shutdown();
        return;
    }
    
    // 初始化类别名称 - 在这里修改类别名称!
    class_names_ = {"zebra", "direction", "obstacle"};
    
    ROS_INFO("YOLO model loaded successfully");
    ROS_INFO("Class name set to: %s", class_names_[0].c_str());
    

    img_sub_ = nh.subscribe("/camera/color/image_raw", 1, 
                           &YoloObstacleNode::imageCallback, this);
    img_pub_ = nh.advertise<sensor_msgs::Image>("/obstacle/image", 1);
    msg_pub_ = nh.advertise<obstacle_detector::PerceptionTargets>("/obstacle/detection", 10);
    direction_pub_ = nh.advertise<std_msgs::String>("/yolo_fastestv2/direction", 1);
    zebra_pub_ = nh.advertise<std_msgs::String>("/yolo_fastestv2/zebra", 1);    
    
    // 初始化帧率计算
    last_time_ = ros::Time::now();
    frame_count_ = 0;
    fps_ = 0.0;
}

YoloObstacleNode::~YoloObstacleNode() {
    ROS_INFO("YoloObstacleNode destroyed");
}

void YoloObstacleNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        // 计算帧率
        frame_count_++;
        ros::Time current_time = ros::Time::now();
        double elapsed = (current_time - last_time_).toSec();
        if (elapsed >= 1.0) {  // 每秒更新一次FPS
            fps_ = frame_count_ / elapsed;
            frame_count_ = 0;
            last_time_ = current_time;
            ROS_INFO("YOLO Detection FPS: %.1f", fps_);
        }
        
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, 
                                                           sensor_msgs::image_encodings::BGR8);
        cv::Mat cvImg = cv_ptr->image;

        std::vector<TargetBox> boxes;
        detector_.detection(cvImg, boxes, 0.8);
        
        for (size_t i = 0; i < boxes.size(); i++) {
            // 1. 创建标签文本（类别 + 置信度百分比）
            char text[256];
            sprintf(text, "%s %.1f%%", class_names_[boxes[i].cate].c_str(), boxes[i].score * 100);
            
            // 2. 计算标签尺寸
            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            
            // 3. 调整标签位置（避免超出边界）
            int x = boxes[i].x1;
            int y = boxes[i].y1 - label_size.height - baseLine;
            if (y < 0) y = 0;
            if (x + label_size.width > cvImg.cols) x = cvImg.cols - label_size.width;
            
            // 4. 绘制白色标签背景
            cv::rectangle(cvImg, 
                         cv::Rect(cv::Point(x, y), 
                                 cv::Size(label_size.width, label_size.height + baseLine)),
                         cv::Scalar(255, 255, 255), -1);
            
            // 5. 绘制黑色标签文字
            cv::putText(cvImg, text, cv::Point(x, y + label_size.height),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            
            // 6. 绘制黄色检测框（或改成绿色 cv::Scalar(0, 255, 0)）
            cv::rectangle(cvImg, cv::Point(boxes[i].x1, boxes[i].y1), 
                         cv::Point(boxes[i].x2, boxes[i].y2), 
                         cv::Scalar(255, 255, 0), 2);

            // ============ 7. 执行白色像素检测（在循环内！） ============
            WhitePixelRatio white_ratio = calculateWhiteRatioInDetectionBox(
                cvImg, boxes[i].x1, boxes[i].y1, boxes[i].x2, boxes[i].y2
            );
            
            ROS_INFO("Box %zu - Left: %.2f%%, Right: %.2f%%, %s", 
                     i, white_ratio.left_ratio * 100, white_ratio.right_ratio * 100,
                     white_ratio.comparison.c_str());

            // 8. 在检测框下方显示白色像素比例
            char ratio_text[256];
            sprintf(ratio_text, "L:%.0f%% R:%.0f%%", 
                    white_ratio.left_ratio * 100, white_ratio.right_ratio * 100);
            
            cv::putText(cvImg, ratio_text, 
                       cv::Point(boxes[i].x1, boxes[i].y2 + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

            // 9. 添加历史记录
            ratio_history.addRatio(white_ratio.left_ratio, white_ratio.right_ratio);
        }  // ✅ for 循环在这里正确结束

        // ============ 发布方向话题（在循环外） ============
        // 当有检测框且比例稳定时，发布方向
        if (!boxes.empty() && ratio_history.isStable()) {
            double avg_left, avg_right;
            ratio_history.getAverageRatios(avg_left, avg_right);
            
            std_msgs::String msg_direction;
            if (avg_left > avg_right) {
                msg_direction.data = "left";
                ROS_INFO("Direction: LEFT (Left ratio: %.2f%% > Right ratio: %.2f%%)", 
                         avg_left * 100, avg_right * 100);
            } else {
                msg_direction.data = "right";
                ROS_INFO("Direction: RIGHT (Right ratio: %.2f%% >= Left ratio: %.2f%%)", 
                         avg_right * 100, avg_left * 100);
            }
            direction_pub_.publish(msg_direction);
            
            // 在图像上显示方向结果
            std::string direction_display = "Direction: " + msg_direction.data;
            cv::putText(cvImg, direction_display, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }

        // ============ 发布结果图像 ============
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), 
                                                           "bgr8", cvImg).toImageMsg();
        out_msg->header = msg->header;
        img_pub_.publish(out_msg);

        // 【修正】与 obstacle_detector.cpp 保持一致的消息格式
        obstacle_detector::PerceptionTargets pub_data;
        
        // 填充消息头（完全对齐 ONNX 版本）
        pub_data.header.stamp = ros::Time::now();
        pub_data.header.frame_id = "obstacle_detection_info";
        pub_data.image_seq = msg->header.seq;
                   // 图像序列号
        
        for (size_t i = 0; i < boxes.size(); i++) {
            obstacle_detector::PerceptionTarget target;
            target.id = static_cast<int>(i);
            target.obj_class = class_names_[boxes[i].cate];  // 使用类成员变量
            target.score = boxes[i].score;
            target.x1 = boxes[i].x1;
            target.y1 = boxes[i].y1;
            target.x2 = boxes[i].x2;
            target.y2 = boxes[i].y2;
            pub_data.targets.push_back(target);
        }
        
        msg_pub_.publish(pub_data);
        
        if (!boxes.empty()) {
            ROS_DEBUG("Detected %zu obstacles", boxes.size());
        }
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

//ncnn 模型加载
int yoloFastestv2::loadModel(const char* paramPath, const char* binPath)
{
    printf("Ncnn mode init:\n%s\n%s\n", paramPath, binPath);

    net.load_param(paramPath);
    net.load_model(binPath);
    
    // 设置线程数
    net.opt.num_threads = numThreads;

    printf("Ncnn model init sucess...\n");

    return 0;
}

float intersection_area(const TargetBox &a, const TargetBox &b)
{
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
    {
        // no intersection
        return 0.f;
    }

    float inter_width = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    float inter_height = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);

    return inter_width * inter_height;
}

bool scoreSort(TargetBox a, TargetBox b) 
{ 
    return (a.score > b.score); 
}

//NMS处理
int yoloFastestv2::nmsHandle(std::vector<TargetBox> &tmpBoxes, 
                             std::vector<TargetBox> &dstBoxes)
{
    std::vector<int> picked;
    
    sort(tmpBoxes.begin(), tmpBoxes.end(), scoreSort);

    for (int i = 0; i < tmpBoxes.size(); i++) {
        int keep = 1;
        for (int j = 0; j < picked.size(); j++) {
            //交集
            float inter_area = intersection_area(tmpBoxes[i], tmpBoxes[picked[j]]);
            //并集
            float union_area = tmpBoxes[i].area() + tmpBoxes[picked[j]].area() - inter_area;
            float IoU = inter_area / union_area;

            if(IoU > nmsThresh && tmpBoxes[i].cate == tmpBoxes[picked[j]].cate) {
                keep = 0;
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
        }
    }
    
    for (int i = 0; i < picked.size(); i++) {
        dstBoxes.push_back(tmpBoxes[picked[i]]);
    }

    return 0;
}

//检测类别分数处理
int yoloFastestv2::getCategory(const float *values, int index, int &category, float &score)
{
    float tmp = 0;
    float objScore  = values[4 * numAnchor + index];

    for (int i = 0; i < numCategory; i++) {
        float clsScore = values[4 * numAnchor + numAnchor + i];
        clsScore *= objScore;

        if(clsScore > tmp) {
            score = clsScore;
            category = i;

            tmp = clsScore;
        }
    }
    
    return 0;
}

//特征图后处理
int yoloFastestv2::predHandle(const ncnn::Mat *out, std::vector<TargetBox> &dstBoxes, 
                              const float scaleW, const float scaleH, const float thresh)
{    //do result
    for (int i = 0; i < numOutput; i++) {   
        int stride;
        int outW, outH, outC;

        outH = out[i].c;
        outW = out[i].h;
        outC = out[i].w;

        assert(inputHeight / outH == inputWidth / outW);
        stride = inputHeight / outH;

        for (int h = 0; h < outH; h++) {
            const float* values = out[i].channel(h);

            for (int w = 0; w < outW; w++) {
                for (int b = 0; b < numAnchor; b++) {                    
                    //float objScore = values[4 * numAnchor + b];
                    TargetBox tmpBox;
                    int category = -1;
                    float score = -1;

                    getCategory(values, b, category, score);

                    if (score > thresh) {
                        float bcx, bcy, bw, bh;

                        bcx = ((values[b * 4 + 0] * 2. - 0.5) + w) * stride;
                        bcy = ((values[b * 4 + 1] * 2. - 0.5) + h) * stride;
                        bw = pow((values[b * 4 + 2] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 0];
                        bh = pow((values[b * 4 + 3] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 1];
                        
                        tmpBox.x1 = (bcx - 0.5 * bw) * scaleW;
                        tmpBox.y1 = (bcy - 0.5 * bh) * scaleH;
                        tmpBox.x2 = (bcx + 0.5 * bw) * scaleW;
                        tmpBox.y2 = (bcy + 0.5 * bh) * scaleH;
                        tmpBox.score = score;
                        tmpBox.cate = category;

                        dstBoxes.push_back(tmpBox);
                    }
                }
                values += outC;
            } 
        } 
    }
    return 0;
}

int yoloFastestv2::detection(const cv::Mat srcImg, std::vector<TargetBox> &dstBoxes, const float thresh)
{   
    dstBoxes.clear();

    float scaleW = (float)srcImg.cols / (float)inputWidth;
    float scaleH = (float)srcImg.rows / (float)inputHeight;
    
    //resize of input image data
    ncnn::Mat inputImg = ncnn::Mat::from_pixels_resize(srcImg.data, ncnn::Mat::PIXEL_BGR,\
                                                       srcImg.cols, srcImg.rows, inputWidth, inputHeight); 

    //Normalization of input image data
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    inputImg.substract_mean_normalize(mean_vals, norm_vals);  

    //creat extractor
    ncnn::Extractor ex = net.create_extractor();

    //set input tensor
    ex.input(inputName, inputImg);

    //forward
    ncnn::Mat out[2]; 
    ex.extract(outputName1, out[0]); //22x22
    ex.extract(outputName2, out[1]); //11x11

    std::vector<TargetBox> tmpBoxes;
    //特征图后处理
    predHandle(out, tmpBoxes, scaleW, scaleH, thresh);

    //NMS
    nmsHandle(tmpBoxes, dstBoxes);
    
    return 0;
}

// ============ 白色像素检测函数实现 ============

/**
 * 对检测矩形执行透视变换并计算白色像素比例
 * @param image 输入图像
 * @param x1 矩形左上角x坐标
 * @param y1 矩形左上角y坐标
 * @param x2 矩形右下角x坐标
 * @param y2 矩形右下角y坐标
 * @return 包含左右上半部分白色像素比例的结果
 */
WhitePixelRatio calculateWhiteRatioInDetectionBox(
    const cv::Mat& image, int x1, int y1, int x2, int y2)
{
    WhitePixelRatio result;
    result.left_ratio = 0.0;
    result.right_ratio = 0.0;
    result.comparison = "Unknown";

    // 验证矩形坐标有效性
    if (x1 < 0 || y1 < 0 || x2 > image.cols || y2 > image.rows || x1 >= x2 || y1 >= y2) {
        ROS_WARN("Invalid detection box coordinates");
        return result;
    }

    // 提取检测矩形区域
    cv::Mat box_img = image(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
    
    // 获取矩形尺寸
    int box_width = box_img.cols;
    int box_height = box_img.rows;
    
    // 定义透视变换的四个源点（矩形的四个角）
    std::vector<cv::Point2f> src_points = {
        cv::Point2f(0, 0),                          // 左上
        cv::Point2f(box_width - 1, 0),              // 右上
        cv::Point2f(0, box_height - 1),             // 左下
        cv::Point2f(box_width - 1, box_height - 1)  // 右下
    };
    
    // 定义透视变换的四个目标点（正面视图，压缩高度）
    float compressed_height = box_height * 0.6f;  // 压缩到60%高度
    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),                              // 左上
        cv::Point2f(box_width - 1, 0),                  // 右上
        cv::Point2f(0, compressed_height),              // 左下
        cv::Point2f(box_width - 1, compressed_height)   // 右下
    };

    // 计算透视变换矩阵
    cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);
    
    // 应用透视变换
    cv::Mat transformed_img;
    cv::warpPerspective(box_img, transformed_img, perspective_matrix, 
                        cv::Size(box_width, static_cast<int>(compressed_height)));

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(transformed_img, hsv, cv::COLOR_BGR2HSV);

    // 定义白色和蓝色的HSV范围
    cv::Scalar lower_white(0, 0, 100);
    cv::Scalar upper_white(180, 30, 255);
    cv::Scalar lower_blue(100, 43, 46);
    cv::Scalar upper_blue(124, 255, 255);

    // 创建掩码
    cv::Mat white_mask, blue_mask, combined_mask;
    cv::inRange(hsv, lower_white, upper_white, white_mask);
    cv::inRange(hsv, lower_blue, upper_blue, blue_mask);
    cv::bitwise_or(white_mask, blue_mask, combined_mask);

    // 形态学优化白色掩码
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(white_mask, white_mask, kernel, cv::Point(-1, -1), 1);
    cv::erode(white_mask, white_mask, kernel, cv::Point(-1, -1), 1);

    // 计算有效像素数（上半部分）
    int mid_height = transformed_img.rows / 2;
    int mid_width = transformed_img.cols / 2;
    
    cv::Mat upper_combined = combined_mask.rowRange(0, mid_height);
    int upper_valid_pixels = cv::countNonZero(upper_combined);

    if (upper_valid_pixels == 0) {
        result.comparison = "No valid pixels in upper half";
        return result;
    }

    // 分割上半部分的左右区域并计算白色占比
    cv::Mat upper_left_combined = upper_combined.colRange(0, mid_width);
    cv::Mat upper_right_combined = upper_combined.colRange(mid_width, transformed_img.cols);
    
    cv::Mat white_upper_left = white_mask.rowRange(0, mid_height).colRange(0, mid_width);
    cv::Mat white_upper_right = white_mask.rowRange(0, mid_height).colRange(mid_width, transformed_img.cols);

    int upper_left_valid = cv::countNonZero(upper_left_combined);
    int upper_right_valid = cv::countNonZero(upper_right_combined);
    int upper_left_white = cv::countNonZero(white_upper_left);
    int upper_right_white = cv::countNonZero(white_upper_right);

    // 计算比例
    result.left_ratio = (upper_left_valid > 0) ? (double)upper_left_white / upper_left_valid : 0.0;
    result.right_ratio = (upper_right_valid > 0) ? (double)upper_right_white / upper_right_valid : 0.0;

    // 比较左右占比
    if (result.left_ratio > result.right_ratio) {
        result.comparison = "Upper-left has higher white ratio";
    } else if (result.right_ratio > result.left_ratio) {
        result.comparison = "Upper-right has higher white ratio";
    } else {
        result.comparison = "Left and right white ratio are equal";
    }

    return result;
}

