#include "yolo-fastestv2.h"
#include <chrono>

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "yolo_obstacle_detector");

    ros::NodeHandle nh;

    
    static const char* class_names[] = {
        "zebra"
        "direction"
        "obstacle"
    };
    
   // 获取模型路径
    std::string pkg_path = ros::package::getPath("obstacle_detector");
    std::string param_path = pkg_path + "/model/test-opt.param";
    std::string bin_path = pkg_path + "/model/test-opt.bin";
    
    // 只创建 YoloObstacleNode 对象（内部会自动订阅和发布）
    YoloObstacleNode node(nh, param_path, bin_path);
    
    ROS_INFO("YOLO obstacle detector started!");
    ros::spin();
    return 0;
}
