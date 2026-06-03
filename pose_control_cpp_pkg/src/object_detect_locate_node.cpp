#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <cmath>
#include <fstream>

struct YoloDetection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Rect box{};
};

class ObjectDetectLocateNode : public rclcpp::Node
{
public:
    ObjectDetectLocateNode()
        : Node("object_detect_locate_node")
    {
        this->declare_parameter<std::string>("camera_name", "table_camera");
        this->declare_parameter<std::string>("target_object", "bottle");
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<double>("confidence_threshold", 0.51);
        this->declare_parameter<double>("nms_threshold", 0.45);
        this->declare_parameter<bool>("use_depth_for_location", true);
        this->declare_parameter<double>("min_red_ratio", 0.05);
        this->declare_parameter<double>("depth_offset", 0.06);
        this->declare_parameter<double>("x_offset", -0.13);
        this->declare_parameter<double>("y_offset", 0.152);

        std::string camera_name = this->get_parameter("camera_name").as_string();
        target_object_ = this->get_parameter("target_object").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        confidence_threshold_ = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
        nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
        min_red_ratio_ = static_cast<float>(this->get_parameter("min_red_ratio").as_double());
        depth_offset_ = this->get_parameter("depth_offset").as_double();
        x_offset_ = this->get_parameter("x_offset").as_double();
        y_offset_ = this->get_parameter("y_offset").as_double();

        if (model_path_.empty()) {
            try {
                model_path_ = ament_index_cpp::get_package_share_directory("pose_control_cpp_pkg") + "/models/yolov8s.onnx";
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to get package share directory: %s", e.what());
                model_path_ = "/home/hl/ros-arm/Week-9-10-Simple-arm-main/pose_control_cpp_pkg/models/yolov8s.onnx";
            }
        }

        RCLCPP_INFO(this->get_logger(), "Object Detect Locate Node Started");
        RCLCPP_INFO(this->get_logger(), "Camera: %s, Target: %s", camera_name.c_str(), target_object_.c_str());
        RCLCPP_INFO(this->get_logger(), "Model: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);

        std::string image_topic = "/" + camera_name + "/image";
        std::string depth_topic = "/" + camera_name + "/depth_image";
        std::string camera_info_topic = "/" + camera_name + "/image/camera_info";

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&ObjectDetectLocateNode::imageCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 10,
            std::bind(&ObjectDetectLocateNode::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&ObjectDetectLocateNode::cameraInfoCallback, this, std::placeholders::_1));

        object_pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/detected_object_position", 10);

        detection_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/yolo_detected_results_image", 10);

        fx_ = 0.0; fy_ = 0.0; cx_ = 0.0; cy_ = 0.0;
        camera_info_received_ = false;
        latest_depth_ = cv::Mat();
        detection_count_ = 0;

        loadYoloModel();
        initClassNames();
    }

private:
    void loadYoloModel()
    {
        std::ifstream file(model_path_);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "YOLOv8 model file not found: %s", model_path_.c_str());
            return;
        }
        file.close();

        try {
            yolo_net_ = cv::dnn::readNetFromONNX(model_path_);
            yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

            if (yolo_net_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv8 model: empty network");
            } else {
                RCLCPP_INFO(this->get_logger(), "YOLOv8 model loaded successfully");
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv8 model: %s", e.what());
        }
    }

    void initClassNames()
    {
        class_names_ = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
            "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
            "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        };

        target_class_id_ = -1;
        if (target_object_ == "coke" || target_object_ == "bottle") {
            target_class_id_ = 39;
            RCLCPP_INFO(this->get_logger(), "Target object 'bottle' (class ID: %d)", target_class_id_);
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (yolo_net_.empty()) {
            return;
        }

        try {
            cv::Mat rgb_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

            std::vector<YoloDetection> detections = runYoloInference(rgb_image);

            std::vector<YoloDetection> filtered_detections;
            if (!detections.empty()) {
                for (auto& det : detections) {
                    if (target_class_id_ >= 0 && det.class_id != target_class_id_) {
                        continue;
                    }

                    if (!filterByRedColor(rgb_image, det.box)) {
                        continue;
                    }

                    filtered_detections.push_back(det);

                    int u = det.box.x + det.box.width / 2;
                    int v = det.box.y + det.box.height / 2;

                    double depth = getDepthAtPoint(u, v);

                    if (depth > 0.0 && camera_info_received_) {
                        geometry_msgs::msg::PointStamped world_point;
                        compute3DPosition(world_point, u, v, depth);

                        object_pose_pub_->publish(world_point);

                        RCLCPP_INFO(this->get_logger(),
                            "Detected %s [ID:%d] at pixel(%d,%d), depth=%.3fm, 3D=(%.3f, %.3f, %.3f)",
                            det.className.c_str(), det.class_id, u, v, depth,
                            world_point.point.x, world_point.point.y, world_point.point.z);
                    }
                }
            }

            publishDebugImage(rgb_image, filtered_detections, msg->header.frame_id, target_class_id_);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv::Mat depth_16u = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                depth_16u.convertTo(latest_depth_, CV_32F, 0.001);
            } else {
                latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            }
            
            if (!latest_depth_.empty()) {
                double min_val, max_val;
                cv::minMaxLoc(latest_depth_, &min_val, &max_val);
                RCLCPP_DEBUG(this->get_logger(), "Depth range: min=%.3f, max=%.3f", min_val, max_val);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert depth image: %s", e.what());
            latest_depth_ = cv::Mat();
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_) {
            fx_ = msg->k[0];
            fy_ = msg->k[4];
            cx_ = msg->k[2];
            cy_ = msg->k[5];
            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                fx_, fy_, cx_, cy_);
        }
    }

    std::vector<YoloDetection> runYoloInference(const cv::Mat& input)
    {
        std::vector<YoloDetection> detections;

        if (yolo_net_.empty()) {
            return detections;
        }

        try {
            cv::Mat modelInput = input;
            int pad_x = 0, pad_y = 0;
            float scale = 1.0;

            cv::Size modelShape(640, 640);
            if (modelShape.width == modelShape.height) {
                modelInput = formatToSquare(modelInput, &pad_x, &pad_y, &scale);
            }

            cv::Mat blob;
            cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
            yolo_net_.setInput(blob);

            std::vector<cv::Mat> outputs;
            yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());

            if (outputs.empty()) {
                return detections;
            }

            int rows = outputs[0].size[1];
            int dimensions = outputs[0].size[2];

            bool yolov8 = false;
            if (dimensions > rows) {
                yolov8 = true;
                rows = outputs[0].size[2];
                dimensions = outputs[0].size[1];
                outputs[0] = outputs[0].reshape(1, dimensions);
                cv::transpose(outputs[0], outputs[0]);
            }

            float* data = (float*)outputs[0].data;

            std::vector<int> class_ids;
            std::vector<float> confidences;
            std::vector<cv::Rect> boxes;

            for (int i = 0; i < rows; ++i) {
                if (yolov8) {
                    float* classes_scores = data + 4;
                    cv::Mat scores(1, class_names_.size(), CV_32FC1, classes_scores);

                    for (size_t j = 0; j < class_names_.size(); ++j) {
                        scores.at<float>(0, j) = 1.0f / (1.0f + exp(-scores.at<float>(0, j)));
                    }

                    cv::Point class_id;
                    double maxClassScore;
                    minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

                    if (maxClassScore > confidence_threshold_) {
                        if (target_class_id_ >= 0 && class_id.x != target_class_id_) {
                            data += dimensions;
                            continue;
                        }

                        confidences.push_back(static_cast<float>(maxClassScore));
                        class_ids.push_back(class_id.x);

                        float x = data[0];
                        float y = data[1];
                        float w = data[2];
                        float h = data[3];

                        int left = int((x - 0.5 * w - pad_x) / scale);
                        int top = int((y - 0.5 * h - pad_y) / scale);
                        int width = int(w / scale);
                        int height = int(h / scale);

                        boxes.push_back(cv::Rect(left, top, width, height));
                    }
                }
                data += dimensions;
            }

            std::vector<int> nms_result;
            cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, nms_result);

            for (size_t i = 0; i < nms_result.size(); ++i) {
                int idx = nms_result[i];

                YoloDetection result;
                result.class_id = class_ids[idx];
                result.confidence = confidences[idx];
                result.className = class_names_[result.class_id];
                result.box = boxes[idx];

                detections.push_back(result);
                detection_count_++;
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in YOLO inference: %s", e.what());
        }

        return detections;
    }

    cv::Mat formatToSquare(const cv::Mat& source, int* pad_x, int* pad_y, float* scale)
    {
        int col = source.cols;
        int row = source.rows;
        int m_inputWidth = 640;
        int m_inputHeight = 640;

        *scale = std::min(m_inputWidth / (float)col, m_inputHeight / (float)row);
        int resized_w = int(col * *scale);
        int resized_h = int(row * *scale);
        *pad_x = (m_inputWidth - resized_w) / 2;
        *pad_y = (m_inputHeight - resized_h) / 2;

        cv::Mat resized;
        cv::resize(source, resized, cv::Size(resized_w, resized_h));
        cv::Mat result = cv::Mat::zeros(m_inputHeight, m_inputWidth, source.type());
        resized.copyTo(result(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));
        return result;
    }

    double getDepthAtPoint(int u, int v)
    {
        if (latest_depth_.empty()) return -1.0;

        int x = std::max(0, std::min(u, latest_depth_.cols - 1));
        int y = std::max(0, std::min(v, latest_depth_.rows - 1));

        float depth = latest_depth_.at<float>(y, x);

        if (std::isnan(depth) || depth <= 0.0) {
            float sum = 0.0;
            int count = 0;
            int window = 5;
            for (int dy = -window; dy <= window; dy += 2) {
                for (int dx = -window; dx <= window; dx += 2) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < latest_depth_.cols && ny >= 0 && ny < latest_depth_.rows) {
                        float d = latest_depth_.at<float>(ny, nx);
                        if (!std::isnan(d) && d > 0.0) {
                            sum += d;
                            count++;
                        }
                    }
                }
            }
            if (count > 0) depth = sum / count;
        }

        return depth;
    }

    void compute3DPosition(geometry_msgs::msg::PointStamped& point, int u, int v, double depth)
    {
        double cam_x = (u - cx_) * depth / fx_;
        double cam_y = (v - cy_) * depth / fy_;
        double cam_z = depth;

        double world_x = cam_z + depth_offset_;
        double world_y = -cam_x + x_offset_;
        double world_z = -cam_y + y_offset_;

        point.header.stamp = this->get_clock()->now();
        point.header.frame_id = "table_camera_link";

        point.point.x = world_x;
        point.point.y = world_y;
        point.point.z = world_z;
    }

    bool filterByRedColor(const cv::Mat& rgb_image, const cv::Rect& box)
    {
        cv::Rect clipped_box = box & cv::Rect(0, 0, rgb_image.cols, rgb_image.rows);
        if (clipped_box.area() == 0) {
            return false;
        }

        cv::Mat roi = rgb_image(clipped_box);
        cv::Mat hsv;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);

        cv::Mat red_mask = mask1 | mask2;
        float red_ratio = static_cast<float>(cv::countNonZero(red_mask)) / static_cast<float>(red_mask.total());

        if (red_ratio < min_red_ratio_) {
            RCLCPP_DEBUG(this->get_logger(), "Filtered out detection with red_ratio=%.3f (< %.3f)",
                        red_ratio, min_red_ratio_);
            return false;
        }

        RCLCPP_DEBUG(this->get_logger(), "Detection passed red filter with ratio=%.3f", red_ratio);
        return true;
    }

    void publishDebugImage(const cv::Mat& rgb_image, const std::vector<YoloDetection>& detections,
                          const std::string& frame_id = "table_camera_link_optical",
                          int target_class_id = -1)
    {
        cv::Mat debug_image = rgb_image.clone();

        int draw_count = 0;
        for (const auto& det : detections) {
            if (target_class_id >= 0 && det.class_id != target_class_id) {
                continue;
            }

            cv::rectangle(debug_image, det.box, cv::Scalar(0, 255, 0), 2);

            std::string text = det.className + " " + std::to_string(det.confidence).substr(0, 4);
            cv::putText(debug_image, text, cv::Point(det.box.x, det.box.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            draw_count++;
        }

        if (draw_count > 0) {
            cv::putText(debug_image, "Target [" + class_names_[target_class_id] + "] detections: " + std::to_string(draw_count),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(debug_image, "Method: YOLOv8",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        } else {
            cv::putText(debug_image, "No target object detected",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv_bridge::CvImage cv_img;
        cv_img.header.frame_id = frame_id;
        cv_img.header.stamp = this->get_clock()->now();
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        cv_img.image = debug_image;

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_img.toImageMsg();
        detection_debug_pub_->publish(*img_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_debug_pub_;

    cv::dnn::Net yolo_net_;
    cv::Mat latest_depth_;
    std::vector<std::string> class_names_;

    bool camera_info_received_;
    double fx_, fy_, cx_, cy_;
    int detection_count_;
    int target_class_id_;

    std::string target_object_;
    std::string model_path_;
    float confidence_threshold_;
    float nms_threshold_;
    float min_red_ratio_;
    double depth_offset_;
    double x_offset_;
    double y_offset_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectLocateNode>();

    RCLCPP_INFO(node->get_logger(), "Object detection node running with YOLOv8. Press Ctrl+C to stop.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}