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

class ProcessNext : public plansys2::ActionExecutorClient
{
public:
  ProcessNext()
  : plansys2::ActionExecutorClient("process_next", 50ms)
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    result_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/marker_result/compressed", 10);
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10, std::bind(&ProcessNext::image_callback, this, std::placeholders::_1));

    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    aruco_params_ = cv::aruco::DetectorParameters::create();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    marker_visible_ = false;
    closest_marker_id_ = -1;
    centered_ = false;
    marker_error_x_ = 0.0;
    
    std::vector<std::string> args = get_arguments();
    
    // process_next has 4 parameters: robot, marker_prev, marker_curr, waypoint
    RCLCPP_INFO(get_logger(), "process_next received %zu arguments", args.size());
    for (size_t i = 0; i < args.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  args[%zu] = %s", i, args[i].c_str());
    }
    
    RCLCPP_INFO(get_logger(), "Looking for closest ArUco marker");
    
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr result_img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  int closest_marker_id_;
  bool marker_visible_;
  bool centered_;
  double marker_error_x_;
  cv::Mat last_image_;
  std::vector<cv::Point2f> closest_marker_corners_;

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
      
      marker_visible_ = false;
      closest_marker_id_ = -1;
      
      if (!ids.empty()) {
        // Find the closest marker (largest size = closest to camera)
        double max_area = -1.0;
        int closest_idx = -1;
        
        for (size_t i = 0; i < ids.size(); ++i) {
          // Calculate marker area as a measure of distance
          cv::Point2f p1 = corners[i][0];
          cv::Point2f p2 = corners[i][2];
          double area = cv::norm(p1 - p2);
          
          if (area > max_area) {
            max_area = area;
            closest_idx = i;
          }
        }
        
        if (closest_idx >= 0) {
          marker_visible_ = true;
          closest_marker_id_ = ids[closest_idx];
          closest_marker_corners_ = corners[closest_idx];
          
          // Calculate center of marker
          cv::Point2f center(0, 0);
          for (const auto& p : closest_marker_corners_) {
            center += p;
          }
          center *= 0.25f;
          
          // Calculate error: distance from image center to marker center
          float image_center_x = image.cols / 2.0f;
          marker_error_x_ = image_center_x - center.x;
          
          RCLCPP_DEBUG(get_logger(), "Detected marker ID %d at x=%.1f, error=%.1f", 
            closest_marker_id_, center.x, marker_error_x_);
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error in image callback: %s", e.what());
    }
  }

  void do_work()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    
    if (last_image_.empty()) {
      RCLCPP_DEBUG(get_logger(), "Waiting for image...");
      return;
    }

    if (marker_visible_) {
      // Check if marker is centered (error < threshold)
      if (std::abs(marker_error_x_) < 15.0) {
        // Marker is centered - take picture and finish
        if (!centered_) {
          centered_ = true;
          RCLCPP_INFO(get_logger(), "Marker centered! Taking picture...");
        }
        
        // Stop rotation
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        
        // Draw and publish the centered marker image
        draw_and_publish();
        
        // Finish the action successfully
        finish(true, 1.0, "Picture Taken Successfully");
      } else {
        // Marker is not centered - rotate to center it
        centered_ = false;
        
        // Simple proportional control for rotation
        // Positive error means marker is to the right, rotate clockwise
        float rotation_speed = 0.003f * marker_error_x_;
        
        // Clamp rotation speed to reasonable values
        if (std::abs(rotation_speed) < 0.1f) {
          rotation_speed = (rotation_speed > 0) ? 0.1f : -0.1f;
        }
        if (std::abs(rotation_speed) > 0.5f) {
          rotation_speed = (rotation_speed > 0) ? 0.5f : -0.5f;
        }
        
        cmd.angular.z = rotation_speed;
        cmd_vel_pub_->publish(cmd);
        
        RCLCPP_DEBUG(get_logger(), "Centering marker ID %d, error=%.1f, rotation=%.2f", 
          closest_marker_id_, marker_error_x_, rotation_speed);
      }
    } else {
      // No marker detected - slow rotation to search
      RCLCPP_DEBUG(get_logger(), "No marker detected, searching...");
      cmd.angular.z = 0.3;
      cmd_vel_pub_->publish(cmd);
      centered_ = false;
    }
  }

  void draw_and_publish()
  {
    if (last_image_.empty() || closest_marker_corners_.empty()) {
      RCLCPP_WARN(get_logger(), "Cannot draw: empty image or corners");
      return;
    }
    
    cv::Mat display_image = last_image_.clone();
    
    // Calculate marker center
    cv::Point2f center(0, 0);
    for (const auto& p : closest_marker_corners_) {
      center += p;
    }
    center *= 0.25f;
    
    // Calculate marker radius
    float radius = cv::norm(closest_marker_corners_[0] - closest_marker_corners_[1]) / 1.5f;
    
    // Draw circle around marker
    cv::circle(display_image, center, (int)radius, cv::Scalar(0, 255, 0), 4);
    
    // Draw marker ID
    cv::putText(display_image, "Marker " + std::to_string(closest_marker_id_), 
      cv::Point(center.x - 30, center.y - radius - 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    
    // Draw crosshair at image center
    int img_cx = display_image.cols / 2;
    cv::line(display_image, cv::Point(img_cx - 20, display_image.rows / 2), 
             cv::Point(img_cx + 20, display_image.rows / 2), cv::Scalar(255, 0, 0), 2);
    cv::line(display_image, cv::Point(img_cx, display_image.rows / 2 - 20),
             cv::Point(img_cx, display_image.rows / 2 + 20), cv::Scalar(255, 0, 0), 2);
    
    // Publish the image
    sensor_msgs::msg::CompressedImage out;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_image).toCompressedImageMsg(out);
    result_img_pub_->publish(out);
    
    RCLCPP_INFO(get_logger(), "Published centered marker image: Marker %d at center", closest_marker_id_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessNext>();
  node->set_parameter(rclcpp::Parameter("action_name", "process_next"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}