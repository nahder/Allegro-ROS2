#include "allegro_hardware_interface/allegro_hardware.hpp"
#include "can_communicator.h"
#include "RockScissorsPaper.h"

namespace allegro_hardware_interface
{

AllegroHardware::AllegroHardware()
: curTime(0.0)
{
  //constructor stuff
}

hardware_interface::CallbackReturn AllegroHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AllegroHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!OpenCAN()) {
    RCLCPP_ERROR(rclcpp::get_logger("AllegroHardware"), "Failed to open CAN communication.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  CreateBHandAlgorithm();
  memset(&vars, 0, sizeof(vars));
  memset(q, 0, sizeof(q));
  memset(q_des, 0, sizeof(q_des));
  memset(tau_des, 0, sizeof(tau_des));
  memset(cur_des, 0, sizeof(cur_des));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AllegroHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < std::size(q); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "allegro_joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &q[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AllegroHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < std::size(q_des); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "allegro_joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &q_des[i]));
  }
  return command_interfaces;
}


hardware_interface::return_type AllegroHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //TODO
  // Update the joint state from the hardware
  // Example: Read from CAN and update `q` array
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AllegroHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //TODO
  // Write the command to the hardware
  // Example: Write desired positions `q_des` to the hardware via CAN
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn AllegroHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!CloseCAN()) {
    RCLCPP_ERROR(rclcpp::get_logger("AllegroHardware"), "Failed to close CAN communication.");
  }

  DestroyBHandAlgorithm();

  RCLCPP_INFO(rclcpp::get_logger("AllegroHardware"), "Cleanup completed successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


}
