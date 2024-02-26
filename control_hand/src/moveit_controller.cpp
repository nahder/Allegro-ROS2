#include "control_hand/can_communicator.h"
#include "control_hand/RockScissorsPaper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_hand/srv/set_joints.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
static const std::string PLANNING_GROUP = "allegro_hand";

class MoveItPlanner : public rclcpp::Node
{
public:
  MoveItPlanner()
  : Node("moveit_planner"), move_group(std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP)
  {

    set_joints_srv = create_service<control_hand::srv::SetJoints>(
      "/set_joints", std::bind(
        &MoveItPlanner::set_joints_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MoveItPlanner::joint_state_callback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
      10ms, std::bind(&MoveItPlanner::timer_callback, this));
  }

private:
  rclcpp::Service<control_hand::srv::SetJoints>::SharedPtr set_joints_srv;
  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp::TimerBase::SharedPtr timer;
  std::vector<double> joint_positions;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

  //service call takes in a vector of joint positions and moves the hand to that position
  //THIS SETS THE JOINTS IN MOVEIT!
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

    if (!in_bounds) {
      RCLCPP_WARN(
        this->get_logger(),
        "clamping to joint limits");
    }
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.move();
    response->success = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    q_des[0] = msg.position[0];
    q_des[1] = msg.position[2];
    q_des[2] = msg.position[1];
    q_des[3] = msg.position[10];
    q_des[4] = msg.position[4];
    q_des[5] = msg.position[12];
    q_des[6] = msg.position[9];
    q_des[7] = msg.position[5];
    q_des[8] = msg.position[8];
    q_des[9] = msg.position[6];
    q_des[10] = msg.position[14];
    q_des[11] = msg.position[3];
    q_des[12] = msg.position[15];
    q_des[13] = msg.position[11];
    q_des[14] = msg.position[13];
    q_des[15] = msg.position[7];
  }


  void timer_callback()
  {
    pBHand->SetMotionType(eMotionType_JOINT_PD);
    double kp[] = {
      500, 800, 900, 500,
      500, 800, 900, 500,
      500, 800, 900, 500,
      1000, 700, 600, 600
    };
    double kd[] = {
      25, 50, 55, 40,
      25, 50, 55, 40,
      25, 50, 55, 40,
      50, 50, 50, 40
    };
    pBHand->SetGainsEx(kp, kd);
  }
};

int main(int argc, char * argv[])
{
  CreateBHandAlgorithm();
  OpenCAN();
  memset(&vars, 0, sizeof(vars));
  memset(q, 0, sizeof(q));
  memset(q_des, 0, sizeof(q_des));
  memset(tau_des, 0, sizeof(tau_des));
  memset(cur_des, 0, sizeof(cur_des));
  curTime = 0.0;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

  CloseCAN();
  DestroyBHandAlgorithm();
  return 0;
}
