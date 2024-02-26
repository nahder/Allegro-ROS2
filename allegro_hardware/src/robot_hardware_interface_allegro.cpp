#include "allegro_hardware_interface/allegro_hardware.hpp"
#include "can_communicator.h"
#include "RockScissorsPaper.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

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


//q is the current state of the joints, q_des is the desired state of the joints
// hw_states_ is updated with latest values from export_state_interfaces
// hw_commands_ is updated with latest values from export_command_interfaces

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

// read current state from actuators
hardware_interface::return_type AllegroHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = q[i];
  }
  return hardware_interface::return_type::OK;
}


// write current command values to actuators
// hw_commands_ is updated with latest values from export_command_interfaces
hardware_interface::return_type AllegroHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < hw_commands_.size(); i++) {
    q_des[i] = hw_commands_[i];
  }
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


} // namespace allegro_hardware_interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  allegro_hardware_interface::AllegroHardware,
  hardware_interface::SystemInterface)
