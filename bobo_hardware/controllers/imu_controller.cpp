#include "bobo_hardware/imu_controller.hpp"

#include <string>

#define ROM_DEBUG 1
namespace bobo_hardware
{
controller_interface::CallbackReturn IMUController::on_init()
{ // OK
  try
  {
    auto_declare<std::vector<std::string>>("inputs", std::vector<std::string>()); // param ရဲ့ inputs 
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// controller_interface::InterfaceConfiguration IMUController::command_interface_configuration() const
// { // config လုပ်ပြီးသားကို return ပေးတယ်။
//   controller_interface::InterfaceConfiguration config;
//   config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // ALL, INDIVIDUAL, NONE
//   config.names = outputs_; // param ထဲက output string တွေသည် IfConfiguration ထဲရောက်သွားမယ်။

//   return config;
// }

controller_interface::InterfaceConfiguration IMUController::state_interface_configuration() const
{ // config လုပ်ပြီးသားကို return ပေးတယ်။
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // ALL, INDIVIDUAL, NONE
  config.names = inputs_; // param ထဲက output string တွေသည် IfConfiguration ထဲရောက်သွားမယ်။

  return config;
}

controller_interface::return_type IMUController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{ // ROM  edit 
    #ifdef ROM_DEBUG
      RCLCPP_INFO(rclcpp::get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "\033 %s (%f)\033[1;0m",state_interfaces_[0].get_name().c_str(),
      state_interfaces_[0].get_value());
    #endif

  double yaw = static_cast<float>(state_interfaces_[0].get_value());
  imu_msg_.header.stamp = this->get_clock()->now();

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  tf2::convert(q, quat);

  imu_msg_.orientation = q;
  // try to publish ...cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
  imu_publisher_->publish(imu_msg_);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn IMUController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // Edited
  try
  {
    inputs_ = get_node()->get_parameter("inputs").as_string_array();   // input paramter ကိုရယူတယ်။ /imu1/yaw_heading

    stop_publisher0_ = get_node()->create_publisher<romCmdType>(
      "~/imu", rclcpp::SystemDefaultsQoS());

    imu_msg_.frame_id = "imu_link"; // this should be equal with urdf and ros2_control xacro
    // Zero-initialize orientation (quaternion)
        imu_msg_.orientation.x = 0.0;
        imu_msg_.orientation.y = 0.0;
        imu_msg_.orientation.z = 0.0;
        imu_msg_.orientation.w = 1.0;  // w should be 1 to represent no rotation

        // Zero-initialize orientation covariance
        imu_msg_.orientation_covariance[0] = 0.0;
        imu_msg_.orientation_covariance[1] = 0.0;
        imu_msg_.orientation_covariance[2] = 0.0;
        imu_msg_.orientation_covariance[3] = 0.0;
        imu_msg_.orientation_covariance[4] = 0.0;
        imu_msg_.orientation_covariance[5] = 0.0;
        imu_msg_.orientation_covariance[6] = 0.0;
        imu_msg_.orientation_covariance[7] = 0.0;
        imu_msg_.orientation_covariance[8] = 0.0;

        // Zero-initialize angular velocity
        imu_msg_.angular_velocity.x = 0.0;
        imu_msg_.angular_velocity.y = 0.0;
        imu_msg_.angular_velocity.z = 0.0;

        // Zero-initialize angular velocity covariance
        imu_msg_.angular_velocity_covariance[0] = 0.0;
        imu_msg_.angular_velocity_covariance[1] = 0.0;
        imu_msg_.angular_velocity_covariance[2] = 0.0;
        imu_msg_.angular_velocity_covariance[3] = 0.0;
        imu_msg_.angular_velocity_covariance[4] = 0.0;
        imu_msg_.angular_velocity_covariance[5] = 0.0;
        imu_msg_.angular_velocity_covariance[6] = 0.0;
        imu_msg_.angular_velocity_covariance[7] = 0.0;
        imu_msg_.angular_velocity_covariance[8] = 0.0;

        // Zero-initialize linear acceleration
        imu_msg_.linear_acceleration.x = 0.0;
        imu_msg_.linear_acceleration.y = 0.0;
        imu_msg_.linear_acceleration.z = 0.0;

        // Zero-initialize linear acceleration covariance
        imu_msg_.linear_acceleration_covariance[0] = 0.0;
        imu_msg_.linear_acceleration_covariance[1] = 0.0;
        imu_msg_.linear_acceleration_covariance[2] = 0.0;
        imu_msg_.linear_acceleration_covariance[3] = 0.0;
        imu_msg_.linear_acceleration_covariance[4] = 0.0;
        imu_msg_.linear_acceleration_covariance[5] = 0.0;
        imu_msg_.linear_acceleration_covariance[6] = 0.0;
        imu_msg_.linear_acceleration_covariance[7] = 0.0;
        imu_msg_.linear_acceleration_covariance[8] = 0.0;
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // Edited
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUController::on_deactivate(
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

}  // namespace bobo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bobo_hardware::IMUController, controller_interface::ControllerInterface)
