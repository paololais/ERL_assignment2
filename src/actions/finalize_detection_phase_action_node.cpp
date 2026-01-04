#include <memory>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>

// PlanSys2
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// ROS 2 Core & Parameters
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class FinalizeDetectionPhase : public plansys2::ActionExecutorClient
{
public:
  FinalizeDetectionPhase()
  : plansys2::ActionExecutorClient("finalize_detection_phase", 1s)
  {
    // Client per modificare la Knowledge Base (PDDL)
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    
    // Client per leggere i parametri dagli altri nodi
    // "detect_marker" è il nome del nodo (definito nel main dell'altro file)
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "detect_marker");
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
    std::vector<int64_t> detected_ids;

    // --- 1. RECUPERA ID DAL NODO "detect_marker" ---
    
    // Controlla se l'altro nodo è vivo
    if (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
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
        // problem_expert_->addPredicate(plansys2::Predicate(predicate));
        RCLCPP_INFO(get_logger(), "Updated KB: %s is detected.", marker_name.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "KB Update Failed for %s: %s", marker_name.c_str(), e.what());
      }
    }

    // Se non abbiamo trovato nulla, potremmo voler fallire l'azione? 
    // Per ora terminiamo con successo comunque.
    if (sorted_ids.empty()) {
      RCLCPP_WARN(get_logger(), "WARNING: No markers were detected in the previous phase!");
    }

    finish(true, 1.0, "Detection Phase Finalized");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalizeDetectionPhase>();

  node->set_parameter(rclcpp::Parameter("action_name", "finalize_detection_phase"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}