#ifndef ALLEGRO_HARDWARE_INTERFACE__ALLEGRO_HARDWARE_HPP_
#define ALLEGRO_HARDWARE_INTERFACE__ALLEGRO_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace allegro_hardware_interface
{
class AllegroHardware : public hardware_interface::SystemInterface
{
public:
  AllegroHardware();

  // override SystemInterface methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
  override;


  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  double allegro_joints_[16];
};

}  // namespace allegro_hardware_interface

#endif  // ALLEGRO_HARDWARE_INTERFACE__ALLEGRO_HARDWARE_HPP_
