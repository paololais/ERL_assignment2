#include <memory>
#include <algorithm>
#include <vector>
#include <string>
#include <cmath>

// PlanSys2 & ROS 2
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ROS Messages
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// OpenCV & CV Bridge
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

class ProcessLastImage : public plansys2::ActionExecutorClient
{
public:
  ProcessLastImage()
  : plansys2::ActionExecutorClient("process_last_image", 50ms)
  {
    // --- Publishers & Subscribers ---
    
    // Command velocity for centering
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Publisher for the final processed image (Requirements)
    result_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/marker_result/compressed", 10);

    // Subscriber for camera input
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10, std::bind(&ProcessLastImage::image_callback, this, std::placeholders::_1));

    // --- ArUco Setup ---
    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_params_ = cv::aruco::DetectorParameters::create();

    // State initialization
    target_id_ = -1;
    is_centered_ = false;
    target_visible_ = false;
  }

  // --- Lifecycle Method: on_activate ---
  // Called when the action starts in the plan.
  // CRITICAL: We parse the arguments here (e.g., "marker_2")
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    // Reset state
    is_centered_ = false;
    target_visible_ = false;
    target_id_ = -1;

    // 1. Retrieve Arguments from PDDL
    // The action in PDDL is likely defined as (process_last_image ?m - marker)
    // arguments_[0] will contain the string name, e.g., "marker_2"
    std::vector<std::string> args = get_arguments();
    
    if (args.empty()) {
      RCLCPP_ERROR(get_logger(), "No arguments provided for ProcessLastImage action!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    std::string marker_name = args[0];
    
    // 2. Extract ID from string (e.g., "marker_2" -> 2)
    try {
      // Assuming format "marker_X" or just "X"
      // Find the last digit(s)
      size_t last_underscore = marker_name.find_last_of('_');
      if (last_underscore != std::string::npos) {
        target_id_ = std::stoi(marker_name.substr(last_underscore + 1));
      } else {
        // Maybe the argument is just the number "2"
        target_id_ = std::stoi(marker_name);
      }
      RCLCPP_INFO(get_logger(), "Targeting Marker ID: %d", target_id_);
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to parse marker ID from argument: %s", marker_name.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
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
  bool is_centered_;
  double current_error_x_; // Distance from center in pixels
  
  // Storage for the last valid image to draw on
  cv::Mat last_image_;
  // Storage for corner points to draw
  std::vector<cv::Point2f> target_corners_;


  // --- IMAGE CALLBACK ---
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

    try {
      cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (image.empty()) return;

      last_image_ = image.clone(); // Keep a copy for drawing later

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, aruco_params_);

      target_visible_ = false;

      if (!ids.empty()) {
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == target_id_) {
            target_visible_ = true;
            target_corners_ = corners[i];

            // Calculate center of the marker
            float cx = (corners[i][0].x + corners[i][1].x + corners[i][2].x + corners[i][3].x) / 4.0;
            float image_center_x = image.cols / 2.0;
            
            // Calculate error
            current_error_x_ = image_center_x - cx;
            break; 
          }
        }
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "CV Error: %s", e.what());
    }
  }

  // --- CONTROL LOOP ---
  void do_work()
  {
    geometry_msgs::msg::Twist cmd;

    // 1. If we haven't processed the image yet (Waiting for camera)
    if (last_image_.empty()) return;

    // 2. If target is visible, try to center it
    if (target_visible_) {
      
      // Check if centered (Deadband tolerance +/- 10 pixels)
      if (std::abs(current_error_x_) < 10.0) {
        
        // --- CENTERED! EXECUTE FINAL TASK ---
        
        // Stop robot
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO(get_logger(), "Target %d Centered. Publishing processed image...", target_id_);
        
        // Draw Graphics (Requirement: Circle + ID)
        draw_and_publish_result();

        // Finish Action
        finish(true, 1.0, "Processed last image completed");
        
      } else {
        // --- P-CONTROLLER for Alignment ---
        // Gain 0.002 for smooth alignment
        float angular_z = 0.002 * current_error_x_;
        
        // Clamp for 4-wheel friction (Deadband fix)
        // If speed is too low, the robot won't move due to friction.
        if (std::abs(angular_z) < 0.15) {
            angular_z = (angular_z > 0) ? 0.15 : -0.15;
        }

        cmd.angular.z = angular_z;
        cmd_vel_pub_->publish(cmd);
        
        send_feedback(0.5, "Aligning with marker...");
      }

    } else {
      // 3. If target NOT visible
      // The robot might be close but pointing slightly wrong. 
      // Rotate slowly to search.
      cmd.angular.z = 0.3;
      cmd_vel_pub_->publish(cmd);
      send_feedback(0.1, "Searching for target in frame...");
    }
  }

  void draw_and_publish_result()
  {
    if (last_image_.empty() || target_corners_.empty()) return;

    // Estimate radius based on corner distance
    float radius = cv::norm(target_corners_[0] - target_corners_[2]) / 2.0;
    
    // Calculate center again for drawing
    cv::Point2f center(0,0);
    for(auto p : target_corners_) center += p;
    center *= 0.25;

    // Draw Green Circle
    cv::circle(last_image_, center, (int)radius + 10, cv::Scalar(0, 255, 0), 4);
    
    // Draw Text
    std::string text = "ID " + std::to_string(target_id_);
    cv::putText(last_image_, text, center - cv::Point2f(0, 20), 
      cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    // Convert back to ROS Message and Publish
    sensor_msgs::msg::CompressedImage out_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", last_image_).toCompressedImageMsg(out_msg);
    
    result_img_pub_->publish(out_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessLastImage>();

  node->set_parameter(rclcpp::Parameter("action_name", "process_last_image"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}