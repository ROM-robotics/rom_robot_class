#ifndef bobo_hardware__GPIO_CONTROLLER_HPP_
#define bobo_hardware__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "controller_interface/controller_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace bobo_hardware
{
using romImuType = sensor_msgs::msg::Imu;  // ROM ADD
using romCmdType = std_msgs::msg::Float32;  // ROM ADD

class GPIOController : public controller_interface::ControllerInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GPIOController);

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;

private:
  std::vector<std::string> inputs_;
  std::vector<std::string> outputs_;

protected:
  void initMsgs();

  // internal commands

  // publisher
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> stop_publisher0_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gpio_publisher1_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gpio_publisher2_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gpio_publisher3_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gpio_publisher4_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_;


  //Original --> control_msgs::msg::InterfaceValue gpio_msg_;
  std_msgs::msg::Float32 estop_msg_; // ROM ADD
  std_msgs::msg::Float32 gpio_msg1_; 
  std_msgs::msg::Float32 gpio_msg2_; 
  std_msgs::msg::Float32 gpio_msg3_; 
  std_msgs::msg::Float32 gpio_msg4_; 
  sensor_msgs::msg::Imu imu_msg_;

   //Original--> std::shared_ptr<CmdType> output_cmd_ptr_;
  std::shared_ptr<romCmdType> command_ptr0; // ROM ADD
  std::shared_ptr<romCmdType> command_ptr1;
  std::shared_ptr<romCmdType> command_ptr2;
  std::shared_ptr<romCmdType> command_ptr3;
  std::shared_ptr<romCmdType> command_ptr4;

  // subscriber
  rclcpp::Subscription<romCmdType>::SharedPtr wheels_estop_command0_;
  rclcpp::Subscription<romCmdType>::SharedPtr subscription_command1_;
  rclcpp::Subscription<romCmdType>::SharedPtr subscription_command2_;
  rclcpp::Subscription<romCmdType>::SharedPtr subscription_command3_;
  rclcpp::Subscription<romCmdType>::SharedPtr subscription_command4_;
};
}  // namespace bobo_hardware

#endif  // bobo_hardware__GPIO_CONTROLLER_HPP_
