#ifndef bobo_hardware__DIFFBOT_SYSTEM_HPP_
#define bobo_hardware__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "bobo_hardware/visibility_control.h"

#include "rom_robot.hpp"

#include "bobo_hardware/gpio_controller.hpp"

namespace bobo_hardware
{
class Stm32Hardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  double timeout_ms = 0;
  int enc_counts_per_rev = 0;
  double pid_p = 0;
  double pid_d = 0;
  double pid_i = 0;
  double pid_o = 0;
  double base_width = 0;

  // for gpios and estop
  int led_max_volt = 0;
  int led_min_volt = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Stm32Hardware);

  bobo_hardware_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  bobo_hardware_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  bobo_hardware_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  bobo_hardware_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  bobo_hardware_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  bobo_hardware_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  bobo_hardware_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  bobo_hardware_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bobo_hardware_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rom_dynamics::Robot bobo_robot;
  
  Config cfg_;

  std::vector<double> hw_gpio_commands;
  std::vector<double> hw_gpio_states;
  
};

}  // namespace bobo_hardware

#endif  // bobo_hardware__DIFFBOT_SYSTEM_HPP_

