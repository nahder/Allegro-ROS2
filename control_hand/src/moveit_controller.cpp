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
    //msg.position is in the order as they are on joint_states topic
    //set q_des by msg.position names (joint_0...joint_15)

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
    RCLCPP_INFO(
      this->get_logger(),
      "q_des:\n"
      "%f, %f, %f, %f,\n"
      "%f, %f, %f, %f,\n"
      "%f, %f, %f, %f,\n"
      "%f, %f, %f, %f",
      q_des[0], q_des[1], q_des[2], q_des[3],
      q_des[4], q_des[5], q_des[6], q_des[7],
      q_des[8], q_des[9], q_des[10], q_des[11],
      q_des[12], q_des[13], q_des[14], q_des[15]);
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
  // OpenCAN();
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

  // CloseCAN();
  DestroyBHandAlgorithm();
  return 0;
}

static double rock[] = {
  -0.1194, //qdes[0]
  1.2068, //qdes[1]
  1.0, //qdes[2]
  1.4042, //qdes[3]
  -0.0093, //qdes[4]
  1.2481, //qdes[5]
  1.4073, //qdes[6]
  0.8163, //qdes[7]
  0.1116,   //qdes[8]
  1.2712, //qdes[9]
  1.3881, //qdes[10]
  1.0122, //qdes[11]
  0.6017, //qdes[12]
  0.2976, //qdes[13]
  0.9034, //qdes[14]
  0.7929};  //qdes[15]

// construct the msg.position vector from q_des using the earlier mapping

// static double msg_position[] = {
//   q_des[0], // -0.1194
//   q_des[2], // 1.0
//   q_des[1], // 1.2068
//   q_des[10], // 1.3881
//   q_des[4], // -0.0093
//   q_des[12], // 0.2976
//   q_des[9], // 1.2712
//   q_des[5], // 1.2481
//   q_des[8], // 0.1116
//   q_des[6], // 1.4073
//   q_des[14], // 0.9034
//   q_des[3], // 1.4042
//   q_des[15], // 0.7929
//   q_des[11], // 0.6017
//   q_des[13], // 0.2976
//   q_des[7] // 0.9034
// };

// static double msg_position_nums[] = {
//   [-0.1194,
//   1.0,
//   1.2068,
//   1.3881,
//   -0.0093,
//   0.2976,
//   1.2712,
//   1.2481,
//   0.1116,
//   1.4073,
//   0.9034,
//   1.4042,
//   0.7929,
//   0.6017,
//   0.2976,
//   0.9034]
// };


// make service call message with all 16 zeros
// ros2 ser
