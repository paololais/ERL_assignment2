#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class ProcessLastImage : public plansys2::ActionExecutorClient
{
public:
  ProcessLastImage()
  : plansys2::ActionExecutorClient("process_last_image", 1s)
  {
  }

private:
  void do_work()
  {
    RCLCPP_INFO(get_logger(), "Processed last image %s executed", get_name());
    finish(true, 1.0, "Processed last image completed");
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
