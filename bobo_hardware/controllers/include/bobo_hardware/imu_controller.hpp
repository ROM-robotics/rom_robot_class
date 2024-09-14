#ifndef bobo_hardware__IMU_CONTROLLER_HPP_
#define bobo_hardware__IMU_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "controller_interface/controller_interface.hpp"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace bobo_hardware
{
  using romCmdType = sensor_msgs::msg::Imu;  // ROM ADD

class IMUController : public controller_interface::ControllerInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IMUController);

  //controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;

private:
  std::vector<std::string> inputs_;   // launch ဖိုင်က inputs ဆိုတဲ့ parameter တွေကို ရယူဖို့
  // parameter တွေကို diffbot_controllers.yaml ကနေ CM ကနေ ယူတယ်လို့ယူဆတယ်။
  // ဒါမှမဟုတ် launch မှာ gpio_contorller_node မှာလို  param ဖိုင် ထည့်ပေးလို့ရပါတယ်။
protected:

  // Parameters for the RRBot simulation ( ros2_control.xacro က လာမည့် param တွေပါ။ လိုတာပြင်ပြီးထည့်ပါ။ )
  // formula ထည့်သင့်တယ်။ 
  /* ခုတော့ diffbot_system ဘက်ပို့လိုက်မယ်။ */
  // double hw_coef_m_;
  // double hw_bias_b_;
  // double hw_period_;
  // std::string link_name_;
  
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_;
  sensor_msgs::msg::Imu imu_msg_;
  // std::vector<double> hw_sensor_states_;  // launch ဖိုင်က inputs ဆိုတဲ့ parameter တွေကို ရယူဖို့
};
}  // namespace bobo_hardware

#endif  // bobo_hardware__IMU_CONTROLLER_HPP_
