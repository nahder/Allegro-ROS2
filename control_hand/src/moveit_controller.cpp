#include "control_hand/can_communicator.h"
#include "control_hand/RockScissorsPaper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_hand/srv/set_joints.hpp"
#include "control_hand/srv/set_config.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


static double nonono1[] = {  //jt 5,9
  -0.37471316618668479, 0.386, 0.174, -0.22753605719833834,
  0.205, 1.636, 1.709, 0.227,
  0.0, 1.636, 1.709, 0.32,
  0.263, 0.410, 0.911, 1.5
};


static double nonono2[] = {  //jt 5,9
  0.37181227113054078, 0.386, 0.174, -0.22753605719833834,
  0.205, 1.636, 1.709, 0.227,
  0.0, 1.636, 1.709, 0.32,
  0.263, 0.410, 0.911, 1.5
};

// double kp[] = {
//   500, 800, 900, 500,
//   500, 800, 900, 500,
//   500, 800, 900, 500,
//   1000, 700, 600, 600
// };
// double kd[] = {
//   25, 50, 55, 40,
//   25, 50, 55, 40,
//   25, 50, 55, 40,
//   50, 50, 50, 40
// };

double kp[] = {
  500, 800, 900, 500,
  500, 800, 900, 500,
  500, 800, 900, 500,
  1000, 700, 600, 600
};

//scale kp down by half
// double kp[] = {
//   250, 400, 450, 250,
//   250, 400, 450, 250,
//   250, 400, 450, 250,
//   500, 350, 300, 300
// };

double kd[] = {
  25, 50, 55, 40,
  25, 50, 55, 40,
  25, 50, 55, 40,
  50, 50, 50, 40
};

// double kd[] = {
//   12.5, 25, 27.5, 20,
//   12.5, 25, 27.5, 20,
//   12.5, 25, 27.5, 20,
//   25, 25, 25, 20
// };

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

    set_config_srv = create_service<control_hand::srv::SetConfig>(
      "/set_config", std::bind(
        &MoveItPlanner::set_config_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MoveItPlanner::joint_state_callback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
      100ms, std::bind(&MoveItPlanner::timer_callback, this));
  }

private:
  rclcpp::Service<control_hand::srv::SetJoints>::SharedPtr set_joints_srv;
  rclcpp::Service<control_hand::srv::SetConfig>::SharedPtr set_config_srv;

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
    move_group.setMaxVelocityScalingFactor(0.9);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.move();
    response->success = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    q_des[0] = msg.position[0]; //joint_0
    q_des[1] = msg.position[2]; //joint_2
    q_des[2] = msg.position[1]; //joint_1
    q_des[3] = msg.position[10]; //joint_10
    q_des[4] = msg.position[4]; //joint_4
    q_des[5] = msg.position[12]; //joint_12
    q_des[6] = msg.position[9]; //joint_9
    q_des[7] = msg.position[5]; //joint_5
    q_des[8] = msg.position[8]; //joint_8
    q_des[9] = msg.position[6]; //joint_6
    q_des[10] = msg.position[14]; //joint_14
    q_des[11] = msg.position[3]; //joint_3
    q_des[12] = msg.position[15]; //joint_15
    q_des[13] = msg.position[11]; //joint_11
    q_des[14] = msg.position[13]; //joint_13
    q_des[15] = msg.position[7]; //joint_7
    pBHand->SetMotionType(eMotionType_JOINT_PD);
    pBHand->SetGainsEx(kp, kd);
  }


  void set_config_callback(
    const std::shared_ptr<control_hand::srv::SetConfig::Request> request,
    std::shared_ptr<control_hand::srv::SetConfig::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "request: %s", request->name.c_str());
    if (request->name == "nonono") {
      move_group.setNamedTarget("Home");
      move_group.setMaxVelocityScalingFactor(1.0);
      move_group.setMaxAccelerationScalingFactor(1.0);
      move_group.move();
      wrong_answer();
      response->success = true;

    } else {
      auto success = move_group.setNamedTarget(request->name);
      if (!success) {
        RCLCPP_WARN(
          this->get_logger(),
          "invalid joint configuration name, moving to default position");
        move_group.setNamedTarget("Home");
      }

      move_group.setMaxVelocityScalingFactor(1.0);
      move_group.setMaxAccelerationScalingFactor(1.0);
      move_group.move();
      response->success = true;
    }
  }

  void timer_callback()
  {
    // pBHand->SetMotionType(eMotionType_JOINT_PD);
    // pBHand->SetGainsEx(kp, kd);
  }

  void wrong_answer()
  {
    //cycle between nonono1,2,3 two times
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 16; j++) {
        q_des[j] = nonono1[j];
      }
      if (pBHand) {pBHand->SetMotionType(eMotionType_JOINT_PD);}
      pBHand->SetGainsEx(kp, kd);
      usleep(0.2 * 1000000);

      for (int j = 0; j < 16; j++) {
        q_des[j] = nonono2[j];
      }
      if (pBHand) {pBHand->SetMotionType(eMotionType_JOINT_PD);}
      pBHand->SetGainsEx(kp, kd);
      usleep(0.2 * 1000000);
    }
    usleep(0.2 * 1000000);
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

  CloseCAN();
  DestroyBHandAlgorithm();
  return 0;
}
