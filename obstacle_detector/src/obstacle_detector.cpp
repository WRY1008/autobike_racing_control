#include "obstacle_detector.h"
#include <fstream>

// 构造函数：初始化 ONNX 模型会话，加载类别文件，配置 ROS 订阅器和发布器
NanoDetPlusNode::NanoDetPlusNode(ros::NodeHandle& nh, const string& model_path, const string& classesFile, float nms_threshold, float objThreshold)
    : score_threshold(objThreshold), nms_threshold(nms_threshold) {
    
    // 加载类别文件
    ifstream ifs(classesFile);
    string line;
    while (getline(ifs, line)) class_names.push_back(line);
    num_class = class_names.size();

    // 配置 ONNX Runtime 会话
    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);
    ort_session = new Ort::Session(env, model_path.c_str(), sessionOptions);

    // 获取模型输入和输出维度
    size_t numInputNodes = ort_session->GetInputCount();
    size_t numOutputNodes = ort_session->GetOutputCount();
    AllocatorWithDefaultOptions allocator;
    for (int i = 0; i < numInputNodes; i++) {
        input_names.push_back(ort_session->GetInputName(i, allocator));
        Ort::TypeInfo input_type_info = ort_session->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();
        input_node_dims.push_back(input_dims);
    }
    for (int i = 0; i < numOutputNodes; i++) {
        output_names.push_back(ort_session->GetOutputName(i, allocator));
        Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();
        output_node_dims.push_back(output_dims);
    }
    inpHeight = input_node_dims[0][2];
    inpWidth = input_node_dims[0][3];
    reg_max = (output_node_dims[0][output_node_dims[0].size() - 1] - num_class) / 4 - 1;

    // 订阅 /image_raw 话题并发布检测结果
    image_sub_ = nh.subscribe("/line_detector/image_raw", 1, &NanoDetPlusNode::imageCallback, this);
    image_pub_ = nh.advertise<sensor_msgs::Image>("/obstacle/image", 1);
	msg_publisher_ = nh.advertise<obstacle_detector::PerceptionTargets>("/obstacle/detection", 10);
}

// 图像回调函数：接收图像消息并执行检测
void NanoDetPlusNode::imageCallback(const obstacle_detector::Copyimage::ConstPtr& msg) {
	sensor_msgs::Image img_msg = msg->image;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
		sensor_msgs::ImageConstPtr img_ptr = boost::make_shared<const sensor_msgs::Image>(img_msg); 
        detect(cv_ptr->image, img_ptr);  // 执行目标检测
        image_pub_.publish(cv_ptr->toImageMsg());  // 发布检测后的图像
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// 目标检测函数：执行模型推理、生成候选框、非极大值抑制，最后绘制检测结果
void NanoDetPlusNode::detect(Mat& srcimg, const sensor_msgs::ImageConstPtr& img_msg) {
    int newh = 0, neww = 0, top = 0, left = 0;
    Mat dst = resize_image(srcimg, &newh, &neww, &top, &left);
    normalize_(dst);
    array<int64_t, 4> input_shape_{1, 3, inpHeight, inpWidth};

    auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Value input_tensor_ = Value::CreateTensor<float>(allocator_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());

    vector<Value> ort_outputs = ort_session->Run(RunOptions{nullptr}, &input_names[0], &input_tensor_, 1, output_names.data(), output_names.size());
    
    vector<BoxInfo> generate_boxes;
    const float* preds = ort_outputs[0].GetTensorMutableData<float>();
    generate_proposal(generate_boxes, preds);
    nms(generate_boxes);

	// 创建用于发布推理结果的ROS Msg
  	obstacle_detector::PerceptionTargets pub_data;

	// 填充消息头
	pub_data.header.stamp = ros::Time::now();
	pub_data.header.frame_id = "obstacle_detection_info";
	pub_data.image_seq = img_msg->header.seq;

    // 绘制检测框
    float ratioh = static_cast<float>(srcimg.rows) / newh;
    float ratiow = static_cast<float>(srcimg.cols) / neww;

	int obstacle_ymax = 0, obstacle_num = 0;

	int zebra_num = 0;

	for (size_t i = 0; i < generate_boxes.size(); ++i) {
        int xmin = static_cast<int>(max((generate_boxes[i].x1 - left) * ratiow, 0.f));
        int ymin = static_cast<int>(max((generate_boxes[i].y1 - top) * ratioh, 0.f));
        int xmax = static_cast<int>(min((generate_boxes[i].x2 - left) * ratiow, static_cast<float>(srcimg.cols)));
        int ymax = static_cast<int>(min((generate_boxes[i].y2 - top) * ratioh, static_cast<float>(srcimg.rows)));
		  if(class_names[generate_boxes[i].label] == "obstacle") {
			  obstacle_num++;
			  if(ymax > obstacle_ymax)
				  obstacle_ymax = ymax;
		    }
		  if(class_names[generate_boxes[i].label] == "zebra") {
			  zebra_num++;
		  }
    }

    for (size_t i = 0; i < generate_boxes.size(); ++i) {
        int xmin = static_cast<int>(max((generate_boxes[i].x1 - left) * ratiow, 0.f));
        int ymin = static_cast<int>(max((generate_boxes[i].y1 - top) * ratioh, 0.f));
        int xmax = static_cast<int>(min((generate_boxes[i].x2 - left) * ratiow, static_cast<float>(srcimg.cols)));
        int ymax = static_cast<int>(min((generate_boxes[i].y2 - top) * ratioh, static_cast<float>(srcimg.rows)));
		    if(class_names[generate_boxes[i].label] == "obstacle") {
			  if(ymax < obstacle_ymax || ((xmin + xmax) / 2 > 400 && obstacle_num > 1))
				  continue;
		    }
		    if(class_names[generate_boxes[i].label] == "zebra") {
			  if((xmin + xmax) / 2 > 500 && zebra_num > 1)
				  continue;
		  }
        rectangle(srcimg, Point(xmin, ymin), Point(xmax, ymax), Scalar(0, 0, 255), 2);
        string label = format("%.2f", generate_boxes[i].score);
        label = class_names[generate_boxes[i].label] + ":" + label;
        putText(srcimg, label, Point(xmin, ymin - 5), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 1);

		  // 使用新的 PerceptionTarget 消息格式
		  obstacle_detector::PerceptionTarget target;
		  target.id = static_cast<int>(i);
		  target.obj_class = class_names[generate_boxes[i].label];
		  target.score = generate_boxes[i].score;
		  target.x1 = xmin;
		  target.y1 = ymin;
		  target.x2 = xmax;
		  target.y2 = ymax;
	  	pub_data.targets.emplace_back(std::move(target));
    }
	msg_publisher_.publish(std::move(pub_data));

	// 发布ROS Msg
    msg_publisher_.publish(std::move(pub_data));
}

// 图像缩放函数：将图像调整为模型输入尺寸
Mat NanoDetPlusNode::resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left) {
    int srch = srcimg.rows, srcw = srcimg.cols;
    *newh = inpHeight;
    *neww = inpWidth;
    Mat dstimg;
    resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
    return dstimg;
}

// 图像归一化函数：将图像像素值缩放至模型输入所需的范围
void NanoDetPlusNode::normalize_(Mat img) {
    int row = img.rows;
    int col = img.cols;
    input_image_.resize(row * col * img.channels());
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                float pix = img.ptr<uchar>(i)[j * 3 + c];
                input_image_[c * row * col + i * col + j] = (pix - mean[c]) / stds[c];
            }
        }
    }
}

//对输入数组进行 softmax 运算，使得每个类别的分数转换为概率
void NanoDetPlusNode::softmax_(const float* x, float* y, int length)
{
	float sum = 0;
	int i = 0;
	for (i = 0; i < length; i++)
	{
		y[i] = exp(x[i]);
		sum += y[i];
	}
	for (i = 0; i < length; i++)
	{
		y[i] /= sum;
	}
}

// 生成候选框
void NanoDetPlusNode::generate_proposal(vector<BoxInfo>& generate_boxes, const float* preds)
{
	const int reg_1max = reg_max + 1;
	const int len = this->num_class + 4 * reg_1max;
	for (int n = 0; n < this->num_stages; n++)
	{
		const int stride_ = this->stride[n];
		const int num_grid_y = (int)ceil((float)this->inpHeight / stride_);
		const int num_grid_x = (int)ceil((float)this->inpWidth / stride_);
		////cout << "num_grid_x=" << num_grid_x << ",num_grid_y=" << num_grid_y << endl;
		
		for (int i = 0; i < num_grid_y; i++)
		{
			for (int j = 0; j < num_grid_x; j++)
			{
				int max_ind = 0;
				float max_score = 0;
				for (int k = 0; k < num_class; k++)
				{
					if (preds[k] > max_score)
					{
						max_score = preds[k];
						max_ind = k;
					}
				}
				if (max_score >= score_threshold)
				{
					const float* pbox = preds + this->num_class;
					float dis_pred[4];
					float* y = new float[reg_1max];
					for (int k = 0; k < 4; k++)
					{
						softmax_(pbox + k * reg_1max, y, reg_1max);
						float dis = 0.f;
						for (int l = 0; l < reg_1max; l++)
						{
							dis += l * y[l];
						}
						dis_pred[k] = dis * stride_;
					}
					delete[] y;
					/*float pb_cx = (j + 0.5f) * stride_ - 0.5;
					float pb_cy = (i + 0.5f) * stride_ - 0.5;*/
					float pb_cx = j * stride_ ;
					float pb_cy = i * stride_;
					float x0 = pb_cx - dis_pred[0];
					float y0 = pb_cy - dis_pred[1];
					float x1 = pb_cx + dis_pred[2];
					float y1 = pb_cy + dis_pred[3];
					generate_boxes.push_back(BoxInfo{ x0, y0, x1, y1, max_score, max_ind });
				}
				preds += len;
			}
		}
	}
	
}

// 非极大值抑制
void NanoDetPlusNode::nms(vector<BoxInfo>& input_boxes)
{
	sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
	vector<float> vArea(input_boxes.size());
	for (int i = 0; i < int(input_boxes.size()); ++i)
	{
		vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1)
			* (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
	}

	vector<bool> isSuppressed(input_boxes.size(), false);
	for (int i = 0; i < int(input_boxes.size()); ++i)
	{
		if (isSuppressed[i]) { continue; }
		for (int j = i + 1; j < int(input_boxes.size()); ++j)
		{
			if (isSuppressed[j]) { continue; }
			float xx1 = (max)(input_boxes[i].x1, input_boxes[j].x1);
			float yy1 = (max)(input_boxes[i].y1, input_boxes[j].y1);
			float xx2 = (min)(input_boxes[i].x2, input_boxes[j].x2);
			float yy2 = (min)(input_boxes[i].y2, input_boxes[j].y2);

			float w = (max)(float(0), xx2 - xx1 + 1);
			float h = (max)(float(0), yy2 - yy1 + 1);
			float inter = w * h;
			float ovr = inter / (vArea[i] + vArea[j] - inter);

			if (ovr >= this->nms_threshold)
			{
				isSuppressed[j] = true;
			}
		}
	}
	// return post_nms;
	int idx_t = 0;
	input_boxes.erase(remove_if(input_boxes.begin(), input_boxes.end(), [&idx_t, &isSuppressed](const BoxInfo& f) { return isSuppressed[idx_t++]; }), input_boxes.end());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detector_node");
    ros::NodeHandle nh;

    // 设置模型和类别文件路径
    std::string model_path = "/home/ros_bike/autobike_ws/src/obstacle_detector/model/nanodet_plus_m_320_model.onnx";
    std::string classesFile = "/home/ros_bike/autobike_ws/src/obstacle_detector/model/coco.names";
    NanoDetPlusNode detector(nh, model_path, classesFile, 0.5, 0.55);

    ros::spin();  // 进入 ROS 事件循环
    return 0;
}
