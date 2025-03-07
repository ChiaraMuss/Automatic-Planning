#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class UnloadFromRobotCarrierMedicalUnit : public plansys2::ActionExecutorClient
{
public:
  UnloadFromRobotCarrierMedicalUnit()
      : plansys2::ActionExecutorClient("unload_from_robot_carrier_medical_unit", 1s)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 1.0; // Quick completion since the duration is short
      send_feedback(progress_, "Unloading box at medical unit");
    } else {
      finish(true, 1.0, "Unloading completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Unloading... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnloadFromRobotCarrierMedicalUnit>();

  node->set_parameter(rclcpp::Parameter("action_name", "unload_from_robot_carrier_medical_unit"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
