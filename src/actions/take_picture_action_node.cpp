#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

class TakePicture : public plansys2::ActionExecutorClient
{
public:
  TakePicture()
  : plansys2::ActionExecutorClient("take_picture", 50ms)
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    result_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/marker_result/compressed", 10);
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10, std::bind(&TakePicture::image_callback, this, std::placeholders::_1));

    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_params_ = cv::aruco::DetectorParameters::create();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    target_visible_ = false;
    target_id_ = -1;
    
    std::vector<std::string> args = get_arguments();
    if (!args.empty()) {
      std::string arg = args[0];
      size_t underscore = arg.find_last_of('_');
      if (underscore != std::string::npos) {
        target_id_ = std::stoi(arg.substr(underscore + 1));
      } else {
        target_id_ = std::stoi(arg);
      }
      RCLCPP_INFO(get_logger(), "Taking Picture of ID: %d", target_id_);
    }
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr result_img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  int target_id_;
  bool target_visible_;
  double current_error_x_;
  cv::Mat last_image_;
  std::vector<cv::Point2f> target_corners_;

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    try {
      cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (image.empty()) return;
      last_image_ = image.clone();
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, aruco_params_);
      target_visible_ = false;
      if (!ids.empty()) {
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == target_id_) {
            target_visible_ = true;
            target_corners_ = corners[i];
            float cx = (corners[i][0].x + corners[i][2].x) / 2.0;
            current_error_x_ = (image.cols / 2.0) - cx;
            break; 
          }
        }
      }
    } catch (...) {}
  }

  void do_work()
  {
    geometry_msgs::msg::Twist cmd;
    if (last_image_.empty()) return;

    if (target_visible_) {
      if (std::abs(current_error_x_) < 10.0) {
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        draw_and_publish();
        finish(true, 1.0, "Picture Taken");
      } else {
        // Friction Clamp Fix
        float az = 0.002 * current_error_x_;
        if (std::abs(az) < 0.15) az = (az > 0) ? 0.15 : -0.15;
        cmd.angular.z = az;
        cmd_vel_pub_->publish(cmd);
      }
    } else {
      cmd.angular.z = 0.3;
      cmd_vel_pub_->publish(cmd);
    }
  }

  void draw_and_publish()
  {
    if (last_image_.empty() || target_corners_.empty()) return;
    cv::Point2f center(0,0);
    for(auto p : target_corners_) center += p;
    center *= 0.25;
    float radius = cv::norm(target_corners_[0] - target_corners_[1]) / 1.5;
    cv::circle(last_image_, center, (int)radius, cv::Scalar(0, 255, 0), 4);
    cv::putText(last_image_, "ID " + std::to_string(target_id_), center, 
      cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2);
    
    sensor_msgs::msg::CompressedImage out;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", last_image_).toCompressedImageMsg(out);
    result_img_pub_->publish(out);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakePicture>();
  node->set_parameter(rclcpp::Parameter("action_name", "take_picture"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}