#include <algorithm>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "assignment2/msg/marker_detection.hpp"

using std::placeholders::_1;

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
  Controller()
  : rclcpp::Node("controller"),
    current_phase_(PlanningPhase::INITIAL),
    plan_active_(false)
  {}

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    marker_sub_ = this->create_subscription<assignment2::msg::MarkerDetection>(
      "/marker_detection", 10,
      std::bind(&Controller::marker_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Controller initialized – starting exploration plan");
  }

  void step()
  {
    if (current_phase_ == PlanningPhase::INITIAL) {
      if (!plan_active_) {
        // Start initial exploration plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
          std::cout << "Could not find plan to reach goal " <<
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          return;
        }

        std::cout << plan.value() << std::endl;
        executor_client_->start_plan_execution(plan.value());
        plan_active_ = true;
      }
      else {
        // Check if plan is finished
        if (!executor_client_->execute_and_check_plan()) {
          auto result = executor_client_->getResult();
          if (result && result.value().success) {
            RCLCPP_INFO(this->get_logger(),
              "Exploration complete - replanning for marker processing");
            RCLCPP_INFO(this->get_logger(), "Detected %zu markers total", 
              detected_markers_.size());

            plan_active_ = false;
            update_marker_predicates_and_goal();
            current_phase_ = PlanningPhase::PROCESSING;
          }
          else {
            RCLCPP_WARN(this->get_logger(), "Plan execution failed");
            plan_active_ = false;
          }
        }
      }
    }
    else if (current_phase_ == PlanningPhase::PROCESSING) {
      if (!plan_active_) {
        // Start marker processing plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
          std::cout << "Could not find plan to reach goal " <<
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          return;
        }

        std::cout << plan.value() << std::endl;
        executor_client_->start_plan_execution(plan.value());
        plan_active_ = true;
      }
      else {
        // Check if plan is finished
        if (!executor_client_->execute_and_check_plan()) {
          auto result = executor_client_->getResult();
          if (result && result.value().success) {
            std::cout << "Everything done!!" << std::endl;
            rclcpp::shutdown();
          }
          else {
            RCLCPP_WARN(this->get_logger(), "Plan execution failed");
            plan_active_ = false;
          }
        }
      }
    }
  }

private:

  enum class PlanningPhase {
    INITIAL,
    PROCESSING
  };

  struct MarkerInfo {
    std::string name;
    int id;
    std::string waypoint;
  };

  void marker_callback(const assignment2::msg::MarkerDetection::SharedPtr msg)
  {
    if (detected_markers_.count(msg->marker_id)) {
      return;  // ignore duplicates
    }

    MarkerInfo info;
    info.name = msg->marker_name;
    info.id = msg->marker_id;
    info.waypoint = msg->waypoint;

    detected_markers_[msg->marker_id] = info;

    RCLCPP_INFO(this->get_logger(),
      "Detected marker: %s (id=%d) at waypoint %s",
      info.name.c_str(), info.id, info.waypoint.c_str());
  }

  void update_marker_predicates_and_goal()
  {
    if (detected_markers_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No markers detected – skipping processing phase");
      return;
    }

    // Sort markers by ID
    std::vector<MarkerInfo> ordered;
    for (const auto & kv : detected_markers_) {
      ordered.push_back(kv.second);
    }

    std::sort(ordered.begin(), ordered.end(),
      [](const MarkerInfo & a, const MarkerInfo & b) {
        return a.id < b.id;
      });

    RCLCPP_INFO(this->get_logger(), "Updating predicates for %zu markers", ordered.size());

    // First marker
    problem_expert_->addPredicate(
      plansys2::Predicate("(is_first " + ordered[0].name + ")"));

    // Precedence chain
    for (size_t i = 0; i + 1 < ordered.size(); ++i) {
      problem_expert_->addPredicate(
        plansys2::Predicate("(precedes " +
          ordered[i].name + " " + ordered[i + 1].name + ")"));
    }

    // Goal
    std::string goal = "(and";
    for (const auto & m : ordered) {
      goal += " (marker_processed " + m.name + ")";
    }
    goal += ")";

    problem_expert_->setGoal(parser::pddl::fromString(goal));
  }

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  rclcpp::Subscription<assignment2::msg::MarkerDetection>::SharedPtr marker_sub_;

  std::map<int, MarkerInfo> detected_markers_;
  PlanningPhase current_phase_;
  bool plan_active_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  node->init();

  rclcpp::Rate rate(5);  // 5 Hz stepping
  while (rclcpp::ok()) {
    node->step();
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();
  return 0;
}