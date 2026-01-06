#include <memory>
#include <algorithm>
#include <cmath>
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

// Custom message
#include "assignment2/msg/marker_detection.hpp"

// OpenCV
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// TF2
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
    // Publishers & Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    detection_pub_ = this->create_publisher<
      assignment2::msg::MarkerDetection>(
      "/marker_detection", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&DetectMarker::odom_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10,
      std::bind(&DetectMarker::image_callback, this, std::placeholders::_1));

    // ArUco setup
    aruco_dict_ = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_ARUCO_ORIGINAL);
    aruco_params_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(get_logger(), "DetectMarker action node initialized");
  }

  //Lifecycle 
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    (void)previous_state;

    total_rotated_ = 0.0;
    first_yaw_read_ = false;
    progress_ = 0.0;

    best_id_ = -1;
    max_area_ = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "START DetectMarker: rotating to find closest marker");

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  // Publishers / Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<
    assignment2::msg::MarkerDetection>::SharedPtr detection_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  // Closest-marker logic
  int best_id_;
  double max_area_;

  // Rotation tracking
  double current_yaw_;
  double prev_yaw_;
  double total_rotated_;
  bool first_yaw_read_;
  float progress_;

  //Odometry
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

  //Image
  void image_callback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    try {
      cv::Mat image =
        cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (image.empty()) return;

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;

      cv::aruco::detectMarkers(
        image, aruco_dict_, corners, ids, aruco_params_);

      for (size_t i = 0; i < ids.size(); ++i) {
        double area = cv::contourArea(corners[i]);

        if (area > max_area_) {
          max_area_ = area;
          best_id_ = ids[i];

          RCLCPP_INFO(
            get_logger(),
            "New closest marker candidate: ID %d (area %.0f)",
            best_id_, max_area_);
        }
      }
    } catch (...) {
      // CV errors
    }
  }

  //Main loop 
  void do_work() override
  {
    if (!first_yaw_read_) return;

    double delta_yaw = current_yaw_ - prev_yaw_;
    while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

    total_rotated_ += std::abs(delta_yaw);
    prev_yaw_ = current_yaw_;

    if (total_rotated_ < 6.4) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = 0.5;
      cmd_vel_pub_->publish(cmd);

      progress_ = std::min(1.0, total_rotated_ / 6.4);
      send_feedback(progress_, "Scanning for markers...");
      return;
    }

    // Stop rotation
    geometry_msgs::msg::Twist stop;
    stop.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);

    //Publish result
    if (best_id_ != -1) {
      auto args = get_arguments();
  

      assignment2::msg::MarkerDetection msg;
      msg.marker_name = args[1];
      msg.waypoint    = args[2];
      msg.marker_id   = best_id_;

      detection_pub_->publish(msg);

      RCLCPP_INFO(
        get_logger(),
        "Detected marker %s at %s with ID %d",
        msg.marker_name.c_str(),
        msg.waypoint.c_str(),
        msg.marker_id);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "DetectMarker finished: no marker detected");
    }

    finish(true, 1.0, "DetectMarker completed");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectMarker>();
  node->set_parameter(
    rclcpp::Parameter("action_name", "detect_marker"));

  node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
