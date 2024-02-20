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
    move_group.setMaxVelocityScalingFactor(0.85);
    move_group.setMaxAccelerationScalingFactor(0.85);
    move_group.move();
    response->success = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState & js_msg)
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data.at(0));
    joint_positions.resize(js_msg.position.size());
    if (joint_positions.size() == 0) {
      joint_positions.at(0) = js_msg.position[0];     // index twist
      joint_positions.at(1) = js_msg.position[1];     // index spread
      joint_positions.at(2) = js_msg.position[2];     // middle spread
      joint_positions.at(3) = js_msg.position[3];     // ring spread
      joint_positions.at(4) = js_msg.position[4];     // little spread
      joint_positions.at(5) = js_msg.position[5];     // thumb spread
      joint_positions.at(6) = js_msg.position[6];     // thumb abd
      joint_positions.at(7) = js_msg.position[7];     // thumb prox
      joint_positions.at(8) = js_msg.position[8];     // thumb med
      joint_positions.at(9) = js_msg.position[9];     // thumb dist
      joint_positions.at(10) = js_msg.position[10];   // index prox
      joint_positions.at(11) = js_msg.position[11];   // index med
      joint_positions.at(12) = js_msg.position[12];   // index dist
      joint_positions.at(13) = js_msg.position[13];   // middle prox
      joint_positions.at(14) = js_msg.position[14];   // middle med
      joint_positions.at(15) = js_msg.position[15];   // middle dist


    }

  }


  void timer_callback()
  {
    //TODO

    //after receiving joint states, move the hand to the joint positions

    // void set_joints(std::vector<double> joint_positions)

    //calll set_joints with joint_positions


    // //log the joint positions
    // RCLCPP_INFO_STREAM(
    //   this->get_logger(), "Joint positions: " << joint_positions.at(
    //     0) << " " << joint_positions.at(1) << " " << joint_positions.at(
    //     2) << " " << joint_positions.at(3) << " " << joint_positions.at(
    //     4) << " " << joint_positions.at(5) << " " << joint_positions.at(
    //     6) << " " << joint_positions.at(7) << " " << joint_positions.at(
    //     8) << " " << joint_positions.at(9) << " " << joint_positions.at(
    //     10) << " " << joint_positions.at(11) << " " << joint_positions.at(
    //     12) << " " << joint_positions.at(13) << " " << joint_positions.at(
    //     14) << " " << joint_positions.at(15));

    if (joint_positions.size() > 0) {
      set_joints(joint_positions);
      //log the joint positions
      RCLCPP_INFO(this->get_logger(), "Joint positions: '%f'", joint_positions.at(0));
      //sleep for 1 second
      rclcpp::sleep_for(1s);
    }

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
