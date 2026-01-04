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
    progress_(0.0)
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
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("your_package_name");
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
      
      // Fallback ai waypoint di default
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

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.header.stamp = now();
      goal_pose.pose.position.x = goal_x_;
      goal_pose.pose.position.y = goal_y_;
      goal_pose.pose.orientation.w = 1.0;

      nav2_msgs::action::NavigateToPose::Goal goal_msg;
      goal_msg.pose = goal_pose;

      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
      options.result_callback =
        [this, target_wp](const auto & result)
        {
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(get_logger(), "Navigation to %s failed", target_wp.c_str());
            finish(false, 1.0, "Navigation failed");
          }
        };

      nav2_client_->async_send_goal(goal_msg, options);

      start_x_ = current_x_;
      start_y_ = current_y_;
      goal_sent_ = true;

      RCLCPP_INFO(get_logger(), "Moving to waypoint %s (%.2f, %.2f)", target_wp.c_str(), goal_x_, goal_y_);
    }

    double total_dist = std::hypot(goal_x_ - start_x_, goal_y_ - start_y_);
    double rem_dist   = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);

    progress_ = (total_dist > 0.0)
      ? 1.0 - std::min(rem_dist / total_dist, 1.0)
      : 1.0;

    send_feedback(progress_, "Moving to " + target_wp);

    if (rem_dist < 1.0) {
      goal_sent_ = false;
      progress_ = 1.0;
      RCLCPP_INFO(get_logger(), "Reached waypoint %s", target_wp.c_str());
      finish(true, 1.0, "Move completed");
    }

    rclcpp::spin_some(nav2_node_);
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

  double start_x_{0.0}, start_y_{0.0};
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