#include "bobo_hardware/external_rrbot_force_torque_sensor.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bobo_hardware
{
hardware_interface::CallbackReturn IMUController::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ပြင်ဖို့လိုမယ်။
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IMUController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // ပြင်ဖို့လိုမယ်။
  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn IMUController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    // ပြင်ဖို့လိုမယ်။
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("IMUController"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("IMUController"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IMUController"), "Successfully activated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ပြင်ဖို့လိုမယ်။
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("IMUController"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("IMUController"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IMUController"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUController::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ပြင်ဖို့လိုမယ်။
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("IMUController"), "Reading...");

  for (uint i = 0; i < hw_sensor_states_.size(); i++)
  {
    // Simulate RRBot's sensor data
    unsigned int seed = time(NULL) + i;
    hw_sensor_states_[i] =
      static_cast<float>(rand_r(&seed)) / (static_cast<float>(RAND_MAX / hw_sensor_change_));
    RCLCPP_INFO(
      rclcpp::get_logger("IMUController"), "Got state %e for sensor %u!",
      hw_sensor_states_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("IMUController"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace bobo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bobo_hardware::IMUController,
  hardware_interface::SensorInterface)