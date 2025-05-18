
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "ros_params_manager.hpp"
#include "ros_params.hpp"
#include "logger.hpp"

static cv::Mat RosImageIntoCvMat(const sensor_msgs::msg::Image::SharedPtr msg, int type = CV_8UC3) {
  const cv::Mat image(msg->height, msg->width, type, const_cast<uint8_t *>(msg->data.data()), msg->step);
  return image;
}
static sensor_msgs::msg::Image CvMatIntoRosImage(const cv::Mat &image) {
  sensor_msgs::msg::Image msg;
  msg.height = image.rows;
  msg.width = image.cols;
  msg.encoding = "bgr8";
  msg.step = image.step[0];
  msg.data.resize(image.total() * image.elemSize());
  std::memcpy(msg.data.data(), image.data, msg.data.size());
  return msg;
}

struct DetectedObject {
  cv::Moments moment;
  double conf;
  cv::Point mass_center;

  void Print() const {
    RCLCPP_INFO(rclcpp::get_logger("DetectedObject"), "DetectedObject: conf = %f", conf);
    RCLCPP_INFO(rclcpp::get_logger("DetectedObject"), "DetectedObject: moment.m00 = %f", moment.m00);
    RCLCPP_INFO(rclcpp::get_logger("DetectedObject"), "DetectedObject: moment.m01 = %f", moment.m01);
    RCLCPP_INFO(rclcpp::get_logger("DetectedObject"), "DetectedObject: moment.m02 = %f", moment.m02);
  }

  bool operator<(const DetectedObject &other) const { return conf < other.conf; }
  bool operator>(const DetectedObject &other) const { return conf > other.conf; }
  bool operator==(const DetectedObject &other) const { return conf == other.conf; }
};

struct ProcessResult {
  std::priority_queue<DetectedObject> detected_objects;
  cv::Mat debug_image;
  std::string debug_image_format;
};

class DartDetectorNode : public rclcpp::Node {
 public:
  DartDetectorNode(rclcpp::NodeOptions options = rclcpp::NodeOptions()
                                                     .allow_undeclared_parameters(true)
                                                     .automatically_declare_parameters_from_overrides(true))
      : Node("dart_detector_node", options) {
    rpm_.Init();
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("debug_image", rclcpp::SensorDataQoS());
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::Image::SharedPtr msg) {
          const cv::Mat cv_image = RosImageIntoCvMat(msg);
          const auto result = ProcessFrame(cv_image);
          bridge_ = cv_bridge::CvImage(msg->header, result.debug_image_format, result.debug_image);
          debug_image_pub_->publish(*bridge_.toImageMsg());
        });
    RCLCPP_INFO(get_logger(), "DartDetectorNode initialized");
  }

 private:
  ProcessResult ProcessFrame(const cv::Mat &image) {
    const cv::Mat green_masked = [&] {
      cv::Mat ret;
      cv::cvtColor(image, ret, cv::COLOR_BGR2HSV);
      cv::Scalar hsv_lo(rpm_.data().hsv_inrange_thresh_lo[0], rpm_.data().hsv_inrange_thresh_lo[1],
                        rpm_.data().hsv_inrange_thresh_lo[2]);
      cv::Scalar hsv_hi(rpm_.data().hsv_inrange_thresh_hi[0], rpm_.data().hsv_inrange_thresh_hi[1],
                        rpm_.data().hsv_inrange_thresh_hi[2]);
      cv::inRange(ret, hsv_lo, hsv_hi, ret);
      return ret;
    }();

    const auto blured = [&] {
      cv::Mat ret;
      const cv::Size blur_size(rpm_.data().gaussian_blur_radius, rpm_.data().gaussian_blur_radius);
      cv::GaussianBlur(green_masked, ret, blur_size, 0);
      return ret;
    }();

    const auto edges = [&] {
      cv::Mat canny_post;
      cv::Canny(blured, canny_post, 128, 200);
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      findContours(canny_post, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
      cv::Mat ret;
      cv::drawContours(ret, contours, -1, cv::Scalar(255), 10);

      // canny again
      ret = cv::Mat::zeros(canny_post.size(), CV_8UC1);
      cv::Canny(ret, ret, 128, 200);
      findContours(canny_post, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
      cv::drawContours(ret, contours, -1, cv::Scalar(255), 10);

      return std::make_pair(ret, contours);
    }();

    const auto ret = [&] {
      ProcessResult result;
      std::vector<DetectedObject> detected_objects;
      for (const auto &contour : edges.second) {
        DetectedObject obj;
        obj.moment = cv::moments(contour);
        obj.mass_center = cv::Point(obj.moment.m10 / obj.moment.m00, obj.moment.m01 / obj.moment.m00);
        obj.conf = obj.moment.m00;
        detected_objects.push_back(obj);
      }
      std::sort(detected_objects.begin(), detected_objects.end(), std::greater<DetectedObject>());
      result.debug_image = image.clone();
      for (const auto &obj : detected_objects) {
        result.detected_objects.push(obj);
        cv::circle(result.debug_image, obj.mass_center, 5, cv::Scalar(255, 0, 0), -1);
        obj.Print();
      }

      result.debug_image_format = "bgr8";
      return result;
    }();

    return ret;
  }

  cv_bridge::CvImage bridge_;
  RosParamsManager<RosParams> rpm_{this};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_{nullptr};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DartDetectorNode>());
  rclcpp::shutdown();
  return 0;
}