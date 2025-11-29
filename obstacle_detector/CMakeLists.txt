cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detector)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 【修改】指定 NCNN 路径
set(ncnn_DIR "/home/ros_bike/ncnn/build/install/lib/cmake/ncnn")
find_package(ncnn REQUIRED)

# 声明消息文件
add_message_files(FILES 
  Point.msg                 # 被 Target 依赖
  Roi.msg                   # 被 Target 依赖
  Target.msg                # 被 PerceptionTargets 依赖（旧版本）
  PerceptionTarget.msg      # 新增（被 PerceptionTargets 依赖）
  PerceptionTargets.msg     # 依赖 PerceptionTarget
  Copyimage.msg             # 独立
)

# 生成消息
generate_messages(DEPENDENCIES 
  std_msgs 
  geometry_msgs 
  sensor_msgs
)

# 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    cv_bridge 
    image_transport
    message_runtime
  DEPENDS 
    OpenCV
)

# ONNX Runtime 路径
set(ONNX_RUNTIME_DIR /home/ros_bike/onnxruntime-linux-x64-1.9.0)

# 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ONNX_RUNTIME_DIR}/include
  ${ncnn_INCLUDE_DIRS}
)

# ONNX Runtime 库路径
link_directories(${ONNX_RUNTIME_DIR}/lib)

# ========== 原有的 NanoDet 节点（ONNX Runtime） ==========
add_executable(obstacle_detector_node 
  src/obstacle_detector.cpp
)

target_link_libraries(obstacle_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ONNX_RUNTIME_DIR}/lib/libonnxruntime.so
)

add_dependencies(obstacle_detector_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ========== 新的 YOLO-FastestV2 节点（NCNN） ==========
add_executable(yolo_obstacles_detector 
  src/yolo_obstacles_detector.cpp
  src/yolo-fastestv2.cpp
)

target_link_libraries(yolo_obstacles_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ncnn
)

add_dependencies(yolo_obstacles_detector 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ========== 安装规则 ==========
install(TARGETS 
  obstacle_detector_node
  yolo_obstacles_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY model/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model
  PATTERN ".svn" EXCLUDE
)