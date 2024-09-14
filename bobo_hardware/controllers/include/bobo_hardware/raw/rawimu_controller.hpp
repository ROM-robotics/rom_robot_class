#ifndef bobo_hardware__IMU_CONTROLLER_HPP_
#define bobo_hardware__IMU_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "controller_interface/controller_interface.hpp"

namespace bobo_hardware
{
  
class IMUController : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IMUController);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the RRBot simulation ( ros2_control.xacro က လာမည့် param တွေပါ။ လိုတာပြင်ပြီးထည့်ပါ။ )
  // formula ထည့်သင့်တယ်။ 
  double hw_coef_m_;
  double hw_bias_b_;
  double hw_period_;
  std::string link_name_;
  // Store the sensor states for the simulated robot
  std::vector<double> hw_sensor_states_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_;
  sensor_msgs::msg::Imu imu_msg_;
};

}  // namespace bobo_hardware

#endif  // bobo_hardware__IMU_CONTROLLER_HPP_