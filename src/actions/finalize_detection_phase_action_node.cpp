#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class FinalizeDetectionPhase : public plansys2::ActionExecutorClient
{
public:
  FinalizeDetectionPhase()
  : plansys2::ActionExecutorClient("finalize_detection_phase", 1s)
  {
  }

private:
  void do_work()
  {
    RCLCPP_INFO(get_logger(), "Finalized detection phase %s executed", get_name());
    finish(true, 1.0, "Finalized detection phase completed");
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
