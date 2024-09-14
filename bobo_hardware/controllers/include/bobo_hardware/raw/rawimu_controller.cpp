#include "bobo_hardware/imu_controller.hpp"

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

  // ပြင်ပြီး။
  hw_coef_m_ = stof(info_.hardware_parameters["hw_coef_m"]);
  hw_bias_b_ = stof(info_.hardware_parameters["hw_bias_b"]);
  hw_period_ = stod(info_.hardware_parameters["hw_period"]);
  link_name_ = info_.hardware_parameters["frame_id"];
  
  // yaw ဆိုတဲ့ state interface ၁ ခုပဲသုံးပါမယ်။
  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IMUController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // "imu" sensor ရဲ့ "yaw" ဆိုတဲ့ state ကို export လုပ်ပါမယ်။ ------------ state မချိန်းရင် publish လုပ်သင့်/ မလုပ်သင့် စဥ်းစားရန်
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
  RCLCPP_INFO(rclcpp::get_logger("IMUController"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // OK
  try
  {
    // reset publisher
    imu_publisher_.reset(); 
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUController::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{----------------------------------------------------------------------------------------------
  // ပြင်ဖို့လိုမယ်။
  
  RCLCPP_INFO(rclcpp::get_logger("IMUController"), "Reading...");


  return hardware_interface::return_type::OK;
}

}  // namespace bobo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bobo_hardware::IMUController,
  hardware_interface::SensorInterface)