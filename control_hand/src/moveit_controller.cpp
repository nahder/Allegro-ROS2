#include "control_hand/can_communicator.h"
#include "control_hand/RockScissorsPaper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_hand/srv/set_joints.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/float64_multi_array.hpp"


using namespace std::chrono_literals;
static const std::string PLANNING_GROUP = "allegro_hand";

class MoveItController : public rclcpp::Node
{
public:
  MoveItController()
  : Node("moveit_controller"), move_group(std::shared_ptr<rclcpp::Node>(
        std::move(this)), PLANNING_GROUP)
  {
    set_joints_srv = create_service<control_hand::srv::SetJoints>(
      "/set_joints", std::bind(
        &MoveItController::set_joints_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    joint_states_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_states", 10,
      std::bind(&MoveItController::joint_states_callback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
      10ms, std::bind(&MoveItController::timer_callback, this));
  }

private:
  rclcpp::Service<control_hand::srv::SetJoints>::SharedPtr set_joints_srv;


  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp::TimerBase::SharedPtr timer;
  std::vector<double> joint_positions;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_states_sub;

  //service call takes in a vector of joint positions and moves the hand to that position
  void set_joints_callback(
    const std::shared_ptr<control_hand::srv::SetJoints::Request> request,
    std::shared_ptr<control_hand::srv::SetJoints::Response> response)
  {
    // convert std::vector<float> to std::vector<double>
    std::vector<double> joint_positions_double(
      request->joint_positions.begin(),
      request->joint_positions.end()
    );
    auto in_bounds = move_group.setJointValueTarget(joint_positions_double);

    // if (!in_bounds) {
    //   RCLCPP_WARN(
    //     this->get_logger(),
    //     "clamping to joint limits");
    // }
    move_group.setMaxVelocityScalingFactor(0.85);
    move_group.setMaxAccelerationScalingFactor(0.85);
    move_group.move();
    response->success = true;
  }

  void joint_states_callback(const std_msgs::msg::Float64MultiArray & msg)
  {
    //TODO

    // joint_positions.at(0) = msg.data.at(12);   // index twist
    // joint_positions.at(1) = msg.data.at(0);   // index base
    // joint_positions.at(2) = msg.data.at(1);
    // joint_positions.at(3) = msg.data.at(15);
    // joint_positions.at(4) = msg.data.at(9);   // middle twist joint 4
    // joint_positions.at(5) = msg.data.at(10);   // joint 5
    // joint_positions.at(6) = msg.data.at(14);   // joint 6
    // joint_positions.at(7) = msg.data.at(7);   // joint 7
    // joint_positions.at(8) = msg.data.at(6);   // ring twist joint 8
    // joint_positions.at(9) = msg.data.at(5);
    // joint_positions.at(10) = msg.data.at(3);
    // joint_positions.at(11) = msg.data.at(2);
    // joint_positions.at(12) = msg.data.at(13);   // thumb wrist joint 12
    // joint_positions.at(13) = msg.data.at(8);   // thumb twist joint 13
    // joint_positions.at(14) = msg.data.at(11);   // joint 14
    // joint_positions.at(15) = msg.data.at(4);   // joint 15
  }


  void timer_callback()
  {
    //TODO

  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
