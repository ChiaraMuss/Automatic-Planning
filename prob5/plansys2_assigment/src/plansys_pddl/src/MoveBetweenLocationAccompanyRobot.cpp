#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveBetweenLocationAccompanyRobot : public plansys2::ActionExecutorClient
{
public:
  MoveBetweenLocationAccompanyRobot()
      : plansys2::ActionExecutorClient("move_between_location_accompany_robot", 250ms)
  {
    progress_ = 0.0;
    add_activation("check_obstacles_node");
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.02;
      send_feedback(progress_, "Moving between locations");
    } else {
      finish(true, 1.0, "Move between locations completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveBetweenLocationAccompanyRobot>();

  node->set_parameter(rclcpp::Parameter("action_name", "move_between_location_accompany_robot"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
