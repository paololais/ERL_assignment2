#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include <ostream>

std::ostream& operator<<(std::ostream& os, const plansys2_msgs::msg::Plan & plan)
{
    os << "Plan:\n";
    for (const auto & item : plan.items) {
        os << "  Action: " << item.action << "\n"
           << "  Duration: " << item.duration << "\n"
           << "  Time: " << item.time << "\n";
    }
    return os;
}

class Controller : public rclcpp::Node
{
public:
  Controller(): rclcpp::Node("controller"),
    plan_in_execution_(false),
    all_actions_done_(false),
    markers_ready_(false),
    current_phase_(PlanningPhase::INITIAL)
  {}

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    // clear detected_ids parameter at start
    //auto params_client = std::make_shared<rclcpp::SyncParametersClient>(this, "detect_marker");
    //params_client->set_parameters({rclcpp::Parameter("detected_ids", std::vector<int64_t>{})});

    // Subscribe to marker order from vision/perception system
    marker_order_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/marker_order", 10,
        std::bind(&Controller::marker_order_callback, this, std::placeholders::_1));

    // Subscribe to action execution feedback
    action_feedback_sub_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        "/action_execution_info", 10,
        std::bind(&Controller::action_feedback_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Controller initialized. Planning initial exploration...");
    
    // Start with initial plan (exploration phase)
    plan();
  }

private:

  enum class PlanningPhase {
    INITIAL,      // Initial exploration
    PROCESSING    // Processing markers in order
  };

  void marker_order_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Expected format: "marker_1 marker_2 marker_3 marker_4"
    std::istringstream iss(msg->data);
    std::string marker;
    
    marker_order_.clear();
    while (iss >> marker) {
      marker_order_.push_back(marker);
    }

    if (marker_order_.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Received marker order with %zu markers:", marker_order_.size());
      for (const auto & m : marker_order_) {
        RCLCPP_INFO(this->get_logger(), "  - %s", m.c_str());
      }
      markers_ready_ = true;
    }
  }

  void plan()
  {
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(this->get_logger(), "Could not find plan to reach goal %s",
        parser::pddl::toString(problem_expert_->getGoal()).c_str());
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Plan found!");
      std::cout << plan.value() << std::endl;
      plan_in_execution_ = true;
      all_actions_done_ = false;
      action_completion_map_.clear();
      executor_client_->start_plan_execution(plan.value());
    }
  }

  void update_marker_predicates()
  {
    if (marker_order_.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "No marker order available!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Updating predicates with marker order...");

    // Set first marker predicate
    std::string first_pred = "(is_first " + marker_order_[0] + ")";
    try {
      plansys2::Predicate pred(first_pred);
      if (problem_expert_->addPredicate(pred)) {
        RCLCPP_INFO(this->get_logger(), "[UPDATE] Set first marker: %s", marker_order_[0].c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to add first marker predicate: %s", e.what());
    }

    // Set precedence chain: marker_order[0] -> marker_order[1] -> ... -> marker_order[n]
    for (size_t i = 0; i < marker_order_.size() - 1; i++) {
      std::string prec_pred = "(precedes " + marker_order_[i] + " " + marker_order_[i + 1] + ")";
      try {
        plansys2::Predicate pred(prec_pred);
        if (problem_expert_->addPredicate(pred)) {
          RCLCPP_INFO(this->get_logger(), "[UPDATE] Set precedence: %s -> %s", 
                     marker_order_[i].c_str(), marker_order_[i + 1].c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to add precedence predicate: %s", e.what());
      }
    }

    // Update goal to process all markers
    std::string goal = "(and";
    for (const auto & marker : marker_order_) {
      goal += " (marker_processed " + marker + ")";
    }
    goal += ")";

    try {
      auto goal_expr = parser::pddl::fromString(goal);
      if (problem_expert_->setGoal(goal_expr)) {
        RCLCPP_INFO(this->get_logger(), "[UPDATE] Goal set to process all markers");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to set goal: %s", e.what());
    }
  }

  void action_feedback_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
  {
    if (msg->action_full_name != ":0") {
      action_completion_map_[msg->action_full_name] = msg->completion;
      
      std::string status_str;
      switch(msg->status) {
        case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
            status_str = "NOT_EXECUTED"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
            status_str = "EXECUTING"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
            status_str = "SUCCEEDED"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
            status_str = "FAILED"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
            status_str = "CANCELLED"; break;
        default:
            status_str = "UNKNOWN"; break;
      }
      
      RCLCPP_INFO(this->get_logger(), 
                 "Action: %s | Completion: %.1f%% | Status: %s",
                 msg->action_full_name.c_str(),
                 msg->completion * 100.0,
                 status_str.c_str());
    }

    // Check if all actions are done
    bool all_done = true;
    for (auto & a : action_completion_map_) {
      if (a.second < 1.0) {
        all_done = false;
        break;
      }
    }

    if (all_done && plan_in_execution_ && action_completion_map_.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Current phase plan completed!");
      plan_in_execution_ = false;
      all_actions_done_ = true;

      // After initial plan completes, check if we should proceed to processing phase
      if (current_phase_ == PlanningPhase::INITIAL && markers_ready_) {
        RCLCPP_INFO(this->get_logger(), "Initial exploration complete. Switching to processing phase...");
        current_phase_ = PlanningPhase::PROCESSING;
        
        // Update predicates and replan for marker processing
        update_marker_predicates();
        
        // Plan the processing phase
        plan();
      }
    }
  }

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_order_sub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr action_feedback_sub_;
  
  std::map<std::string, float> action_completion_map_;
  std::vector<std::string> marker_order_;
  
  bool plan_in_execution_;
  bool all_actions_done_;
  bool markers_ready_;
  PlanningPhase current_phase_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Controller>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}