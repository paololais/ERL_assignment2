#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <cmath>
#include <map>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

struct Waypoint {
  double x;
  double y;
};

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("navigate", 500ms),
    goal_sent_(false),
    progress_(0.0),
    retry_count_(0),
    max_retries_(3)
  {
    // load waypoints from YAML file
    load_waypoints();

    odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 
      10,
      std::bind(&MoveAction::odom_callback, this, std::placeholders::_1)
    );

    nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      nav2_node_,
      "navigate_to_pose"
    );
  }

private:
  void load_waypoints()
  {
    try {
      // package path
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("assignment2");
      std::string yaml_file = package_share_dir + "/config/waypoints.yaml";
      
      RCLCPP_INFO(get_logger(), "Loading waypoints from: %s", yaml_file.c_str());
      
      // Load YAML file
      YAML::Node config = YAML::LoadFile(yaml_file);
      
      if (config["waypoints"]) {
        for (const auto& wp_node : config["waypoints"]) {
          std::string wp_name = wp_node.first.as<std::string>();
          double x = wp_node.second["x"].as<double>();
          double y = wp_node.second["y"].as<double>();
          
          waypoints_[wp_name] = {x, y};
          
          RCLCPP_INFO(get_logger(), "Loaded waypoint %s: (%.2f, %.2f)", wp_name.c_str(), x, y);
        }
      }
      
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints: %s", e.what());
      RCLCPP_WARN(get_logger(), "Using default waypoints as fallback");
      
      // Fallback to default waypoints
      waypoints_["wp_1"] = {-6.0, -6.0};
      waypoints_["wp_2"] = {-6.0,  6.0};
      waypoints_["wp_3"] = { 6.0, -6.0};
      waypoints_["wp_4"] = { 6.0,  6.0};
    }
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 3) {
      RCLCPP_ERROR(get_logger(), "move: not enough arguments");
      finish(false, 0.0, "Invalid arguments");
      return;
    }

    const std::string target_wp = args[2];

    if (!get_waypoint_coordinates(target_wp, goal_x_, goal_y_)) {
      RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", target_wp.c_str());
      finish(false, 0.0, "Unknown waypoint");
      return;
    }

    if (!goal_sent_) {
      if (!nav2_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(get_logger(), "NavigateToPose server not available");
        return;
      }

      send_nav2_goal(target_wp);
      goal_sent_ = true;
      retry_count_ = 0;
    }

    rclcpp::spin_some(nav2_node_);
  }

  void send_nav2_goal(const std::string& target_wp)
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = now();
    goal_pose.pose.position.x = goal_x_;
    goal_pose.pose.position.y = goal_y_;
    goal_pose.pose.orientation.w = 1.0;

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
    
    // Feedback callback - called during navigation
    options.feedback_callback =
      [this, target_wp](
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
      {
        // Update progress based on navigation feedback
        if (feedback) {
          // distance_remaining is the remaining distance to goal in meters
          if (feedback->distance_remaining > 0.0) {
            // Calculate progress: 1.0 means goal reached, 0.0 means just started
            // Assuming max 20m is full distance
            progress_ = std::max(0.0, std::min(1.0, 1.0 - (feedback->distance_remaining / 20.0)));
            send_feedback(progress_, "Moving to " + target_wp + 
                         " (distance remaining: " + std::to_string(feedback->distance_remaining) + "m)");
          }
        }
      };

    // Result callback - called when goal completes (success or failure)
    options.result_callback =
      [this, target_wp](const auto & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          progress_ = 1.0;
          RCLCPP_INFO(get_logger(), "Successfully reached waypoint %s", target_wp.c_str());
          goal_sent_ = false;
          finish(true, 1.0, "Move completed successfully");
        }
        else if (result.code == rclcpp_action::ResultCode::ABORTED || 
                 result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_WARN(get_logger(), "Navigation to %s failed (code: %d), retrying...", 
                     target_wp.c_str(), static_cast<int>(result.code));
          
          if (retry_count_ < max_retries_) {
            retry_count_++;
            RCLCPP_WARN(get_logger(), "Retry attempt %d/%d", retry_count_, max_retries_);
            send_nav2_goal(target_wp);
          }
          else {
            RCLCPP_ERROR(get_logger(), "Max retries exceeded for waypoint %s", target_wp.c_str());
            goal_sent_ = false;
            finish(false, progress_, "Navigation failed after retries");
          }
        }
      };

    nav2_client_->async_send_goal(goal_msg, options);
    RCLCPP_INFO(get_logger(), "Sending goal to navigate to waypoint %s (%.2f, %.2f)", 
               target_wp.c_str(), goal_x_, goal_y_);
  }

  bool get_waypoint_coordinates(const std::string & wp, double & x, double & y) {
    auto it = waypoints_.find(wp);
    if (it != waypoints_.end()) {
      x = it->second.x;
      y = it->second.y;
      return true;
    }
    return false;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  bool goal_sent_;
  float progress_;
  int retry_count_;
  int max_retries_;

  double current_x_{0.0}, current_y_{0.0};
  double goal_x_{0.0}, goal_y_{0.0};

  std::map<std::string, Waypoint> waypoints_;

  rclcpp::Node::SharedPtr nav2_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "navigate"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}