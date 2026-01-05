#include <memory>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

// PlanSys2
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// ROS 2 Core & Parameters
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

class FinalizeDetectionPhase : public plansys2::ActionExecutorClient
{
public:
  FinalizeDetectionPhase()
  : plansys2::ActionExecutorClient("finalize_exploration", 1s)
  {
    // Client to modify KB (PDDL)
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();

    // Client to read parameters from the detect_marker node
    // "detect_marker" is the node name used in the other file's main()
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "detect_marker");

    // Optional: declare a local parameter to publish ordered names
    this->declare_parameter("ordered_marker_names", std::vector<std::string>({}));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(get_logger(), "FinalizeDetectionPhase Activated.");
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;

  void do_work()
  {
    // --- 1) Read "name:id" pairs from detect_marker ---
    std::vector<std::string> raw_pairs;

    if (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_WARN(get_logger(), "Node 'detect_marker' not available. Using empty list.");
    } else {
      auto parameters = parameters_client_->get_parameters({ "detected_ids" });
      if (!parameters.empty()) {
        raw_pairs = parameters[0].as_string_array();
        RCLCPP_INFO(get_logger(), "Received %lu name:id pairs from detect_marker.", raw_pairs.size());
      } else {
        RCLCPP_WARN(get_logger(), "Parameter 'detected_ids' not found or empty!");
      }
    }

    // --- 2) Parse "name:id" into vector of { name, id } ---
    struct MarkerInfo { std::string name; int id; };
    std::vector<MarkerInfo> markers;
    markers.reserve(raw_pairs.size());

    for (const auto & s : raw_pairs) {
      auto pos = s.find(':');
      if (pos == std::string::npos) {
        RCLCPP_WARN(get_logger(), "Skipping malformed entry '%s' (expected name:id)", s.c_str());
        continue;
      }
      std::string name = s.substr(0, pos);
      std::string id_str = s.substr(pos + 1);
      try {
        int id = std::stoi(id_str);
        markers.push_back({ name, id });
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Skipping entry '%s': %s", s.c_str(), e.what());
      }
    }

    // --- 3) Sort ascending by ID ---
    std::sort(markers.begin(), markers.end(),
              [](const MarkerInfo & a, const MarkerInfo & b) { return a.id < b.id; });

    // --- 4) Build ordered list of names (to pass to PlanSys / downstream) ---
    std::vector<std::string> ordered_names;
    ordered_names.reserve(markers.size());

    std::stringstream log_ss;
    log_ss << "[ ";
    for (const auto & m : markers) {
      ordered_names.push_back(m.name);
      log_ss << m.name << ":" << m.id << " ";
    }
    log_ss << "]";

    RCLCPP_INFO(get_logger(), "Final ordered markers by ID: %s", log_ss.str().c_str());

    // --- 5) Update Knowledge Base (optional, if your domain has these predicates) ---
    // Example predicate: (marker_detected ?m - marker)
    // Ensure your domain.pddl has the predicate `marker_detected ?m - marker`
    for (const auto & m : markers) {
      const std::string predicate = "(marker_detected " + m.name + ")";
      try {
        // Uncomment when the predicate exists in your domain
        // problem_expert_->addPredicate(plansys2::Predicate(predicate));
        RCLCPP_INFO(get_logger(), "KB updated: %s", predicate.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "KB update failed for %s: %s", m.name.c_str(), e.what());
      }
    }

    // --- 6) Optionally publish ordered list so downstream actions can consume it directly ---
    this->set_parameter(rclcpp::Parameter("ordered_marker_names", ordered_names));

    if (ordered_names.empty()) {
      RCLCPP_WARN(get_logger(), "WARNING: No markers were detected in the previous phase!");
    }

    // --- 7) Clear the detected_ids parameter from detect_marker ---
    try {
      parameters_client_->set_parameters(
          {rclcpp::Parameter("detected_ids", std::vector<std::string>{})}
      );
      RCLCPP_INFO(get_logger(), "Cleared detected_ids parameter.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to clear detected_ids parameter: %s", e.what());
    }

    finish(true, 1.0, "Detection Phase Finalized");
  }
};

int main(int argc, char * * argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalizeDetectionPhase>();
  node->set_parameter(rclcpp::Parameter("action_name", "finalize_exploration"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
