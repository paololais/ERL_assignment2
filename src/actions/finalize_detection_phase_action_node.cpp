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
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

class FinalizeDetectionPhase : public plansys2::ActionExecutorClient
{
public:
  FinalizeDetectionPhase()
  : plansys2::ActionExecutorClient("finalize_exploration", 1s)
  {
    // Client per modificare la Knowledge Base (PDDL)
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    
    // Client per leggere i parametri dagli altri nodi
    // "detect_marker" è il nome del nodo (definito nel main dell'altro file)
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "detect_marker");
    
    // Publisher per inviare l'ordine dei marker al Controller
    marker_order_pub_ = this->create_publisher<std_msgs::msg::String>("/marker_order", 10);
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_order_pub_;

  void do_work()
  {
    std::vector<int64_t> detected_ids;

    // --- 1. RECUPERA ID DAL NODO "detect_marker" ---
    
    // Controlla se l'altro nodo è vivo
    if (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        finish(false, 0.0, "Service interrupted");
        return;
      }
      RCLCPP_WARN(get_logger(), "Node 'detect_marker' not available. Using empty list.");
    } else {
      // Richiedi il parametro "detected_ids"
      auto parameters = parameters_client_->get_parameters({"detected_ids"});
      
      if (!parameters.empty()) {
        detected_ids = parameters[0].as_integer_array();
        RCLCPP_INFO(get_logger(), "Received %lu IDs from detect_marker node.", detected_ids.size());
      } else {
        RCLCPP_WARN(get_logger(), "Parameter 'detected_ids' not found or empty!");
      }
    }

    // --- 2. ORDINAMENTO (Richiesto dall'assignment) ---
    // Copiamo in un vector<int> standard per comodità
    std::vector<int> sorted_ids;
    for(auto id : detected_ids) {
      sorted_ids.push_back((int)id);
    }

    // Ordina in senso crescente
    std::sort(sorted_ids.begin(), sorted_ids.end());

    // Log per debug
    std::stringstream ss;
    for(int id : sorted_ids) ss << id << " ";
    RCLCPP_INFO(get_logger(), "Final Sorted Sequence: [ %s]", ss.str().c_str());

    // --- 3. AGGIORNA KNOWLEDGE BASE PDDL ---
    // Diciamo al planner che questi marker sono "pronti"
    for (int id : sorted_ids) {
      std::string marker_name = "marker_" + std::to_string(id);
      
      // Esempio predicato: (marker_detected marker_1)
      // Assicurati che questo predicato esista nel tuo domain.pddl
      std::string predicate = "(marker_detected " + marker_name + ")";
      
      try {
        if (problem_expert_->addPredicate(plansys2::Predicate(predicate))) {
          RCLCPP_INFO(get_logger(), "Updated KB: %s is detected.", marker_name.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "KB Update Failed for %s: %s", marker_name.c_str(), e.what());
      }
    }

    // --- 4. PUBBLICA L'ORDINE DEI MARKER AL CONTROLLER ---
    if (!sorted_ids.empty()) {
      std::stringstream marker_order_stream;
      for (size_t i = 0; i < sorted_ids.size(); i++) {
        marker_order_stream << "marker_" << sorted_ids[i];
        if (i < sorted_ids.size() - 1) {
          marker_order_stream << " ";
        }
      }
      
      std_msgs::msg::String msg;
      msg.data = marker_order_stream.str();
      marker_order_pub_->publish(msg);
      
      RCLCPP_INFO(get_logger(), "Published marker order: %s", msg.data.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "WARNING: No markers were detected in the previous phase!");
      finish(false, 0.0, "No markers detected");
      return;
    }

    finish(true, 1.0, "Detection Phase Finalized and Marker Order Published");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalizeDetectionPhase>();

  node->set_parameter(rclcpp::Parameter("action_name", "finalize_exploration"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}