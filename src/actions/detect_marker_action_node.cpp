#include <memory>
#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

// PlanSys2 & ROS 2
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"

// ROS Messages
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// OpenCV & CV Bridge
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// TF2 (Quaternion to Euler)
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class DetectMarker : public plansys2::ActionExecutorClient
{
public:
  DetectMarker()
  : plansys2::ActionExecutorClient("detect_marker", 50ms)
  {
    // --- Publishers & Subscribers ---
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DetectMarker::odom_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10, std::bind(&DetectMarker::image_callback, this, std::placeholders::_1));

    // --- ArUco Setup ---
    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_params_ = cv::aruco::DetectorParameters::create();

    // Declare parameter to share found IDs with other nodes
    this->declare_parameter("detected_ids", std::vector<int64_t>({}));
  }

  // --- Lifecycle: on_activate ---
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    // Reset state variables
    total_rotated_ = 0.0;
    first_yaw_read_ = false;
    detected_ids_.clear();
    progress_ = 0.0;
    
    RCLCPP_INFO(get_logger(), "DetectMarker Action Started. Rotating 360...");
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
  std::set<int> detected_ids_;

  double current_yaw_;
  double prev_yaw_;
  double total_rotated_;
  bool first_yaw_read_;
  float progress_;

  // --- Odometry Callback ---
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;

    if (!first_yaw_read_) {
      prev_yaw_ = current_yaw_;
      first_yaw_read_ = true;
    }
  }

  // --- Image Callback ---
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

    try {
      cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (image.empty()) return;

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, aruco_params_);

      if (!ids.empty()) {
        for (int id : ids) {
          if (detected_ids_.find(id) == detected_ids_.end()) {
            detected_ids_.insert(id);
            RCLCPP_INFO(get_logger(), "Found Marker ID: %d", id);
          }
        }
      }
    } catch (...) {}
  }

  // --- Main Loop ---
  void do_work()
  {
    if (!first_yaw_read_) return;

    // Calculate rotation delta handling wrap-around
    double delta_yaw = current_yaw_ - prev_yaw_;
    while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

    total_rotated_ += std::abs(delta_yaw);
    prev_yaw_ = current_yaw_;

    // Check if full rotation (approx 6.28 rad) is done. Using 6.4 for overlap.
    if (total_rotated_ < 6.4) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = 0.5; // High speed for skid-steer
      cmd_vel_pub_->publish(cmd);
      
      progress_ = std::min(1.0, total_rotated_ / 6.4);
      send_feedback(progress_, "Scanning environment...");
    } else {
      // Stop
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);

      // Save detected IDs to parameter for the next node
      std::vector<int64_t> ids_vec(detected_ids_.begin(), detected_ids_.end());
      this->set_parameter(rclcpp::Parameter("detected_ids", ids_vec));

      RCLCPP_INFO(get_logger(), "Scan Complete. Total markers: %lu", detected_ids_.size());
      finish(true, 1.0, "DetectMarker completed");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectMarker>();
  node->set_parameter(rclcpp::Parameter("action_name", "detect_marker"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}