#include "bobo_hardware/gpio_controller.hpp"

#include <string>

namespace bobo_hardware
{
controller_interface::CallbackReturn GPIOController::on_init()
{ // Edited
  try
  {
    auto_declare<std::vector<std::string>>("inputs", std::vector<std::string>()); // ?
    auto_declare<std::vector<std::string>>("outputs", std::vector<std::string>());

    //auto_declare<std::vector<std::string>>("inputs", std::vector<std::string>());
    //auto_declare<std::vector<std::string>>("outputs", std::vector<std::string>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{ // Edited
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = outputs_;

  return config;
}

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
{ // Edited
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = inputs_;

  return config;
}

controller_interface::return_type GPIOController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{ 
  RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: (%f)", state_interfaces_[0].get_name().c_str(),
      state_interfaces_[0].get_value());

  estop_msg_.data = static_cast<float>(state_interfaces_[0].get_value());
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mESTOP\033[1;0m"), "\033[1;35m %f \033[1;0m", estop_msg_.data);
  stop_publisher0_->publish(estop_msg_);

  gpio_msg1_.data = static_cast<float>(state_interfaces_[1].get_value());
  gpio_publisher1_->publish(gpio_msg1_);

  gpio_msg2_.data = static_cast<float>(state_interfaces_[2].get_value());
  gpio_publisher2_->publish(gpio_msg2_);

  gpio_msg3_.data = static_cast<float>(state_interfaces_[3].get_value());
  gpio_publisher3_->publish(gpio_msg3_);

  gpio_msg4_.data = static_cast<float>(state_interfaces_[4].get_value());
  gpio_publisher4_->publish(gpio_msg4_);

  double yaw = static_cast<float>(state_interfaces_[5].get_value());
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mget_value_yaw(gpio)\033[1;0m"), "\033[1;35m %.4f \033[1;0m", yaw);
  imu_msg_.header.stamp = get_node()->get_clock()->now();//-------------------------------cccccccccccccccc

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw); //radian

  imu_msg_.orientation.x = q.x();
  imu_msg_.orientation.y = q.y();
  imu_msg_.orientation.z = q.z();
  imu_msg_.orientation.w = q.w();
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mget_value_rotation(gpio)\033[1;0m"), "\033[1;35m %.4f \033[1;0m", imu_msg_.orientation.w);
  //try to publish ...cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
  imu_publisher_->publish(imu_msg_);
 

  //add something 

  if (command_ptr0 != nullptr) {
    command_interfaces_[0].set_value(command_ptr0->data);
  }
  if (command_ptr1 != nullptr) {
    command_interfaces_[1].set_value(command_ptr1->data);
    RCLCPP_DEBUG(get_node()->get_logger(), "%s: (%f)", command_interfaces_[1].get_name().c_str(), command_interfaces_[1].get_value());
  }
  if (command_ptr2 != nullptr) {
    command_interfaces_[2].set_value(command_ptr2->data);
  }
  if (command_ptr3 != nullptr) {
    command_interfaces_[3].set_value(command_ptr3->data);
  }
  if (command_ptr4 != nullptr) {
    command_interfaces_[4].set_value(command_ptr4->data);
  }


  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GPIOController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // Edited
  try
  {
    inputs_ = get_node()->get_parameter("inputs").as_string_array();
    outputs_ = get_node()->get_parameter("outputs").as_string_array();

    initMsgs();

    // register publisher
    // Original --> gpio_publisher_ = get_node()->create_publisher<control_msgs::msg::InterfaceValue>(
    //   "~/inputs", rclcpp::SystemDefaultsQoS());

    stop_publisher0_ = get_node()->create_publisher<romCmdType>(
      "~/estop/status", rclcpp::SystemDefaultsQoS());
    gpio_publisher1_ = get_node()->create_publisher<romCmdType>(
      "~/led1/status",  rclcpp::SystemDefaultsQoS());
    gpio_publisher2_ = get_node()->create_publisher<romCmdType>(
      "~/led2/status",  rclcpp::SystemDefaultsQoS());
    gpio_publisher3_ = get_node()->create_publisher<romCmdType>(
      "~/led3/status",  rclcpp::SystemDefaultsQoS());
    gpio_publisher4_ = get_node()->create_publisher<romCmdType>(
      "~/led4/status",  rclcpp::SystemDefaultsQoS());
    imu_publisher_ = get_node()->create_publisher<romImuType>(
      "~/imu", rclcpp::SystemDefaultsQoS());


    // register subscriber
    // Original --> subscription_command_ = get_node()->create_subscription<CmdType>(
    //   "~/commands", rclcpp::SystemDefaultsQoS(),
    //   [this](const CmdType::SharedPtr msg) { output_cmd_ptr_ = msg; });

    wheels_estop_command0_ = get_node()->create_subscription<romCmdType>(
      "~/estop/command", rclcpp::SystemDefaultsQoS(),
      [this](const romCmdType::SharedPtr msg) { command_ptr0 = msg; });
    subscription_command1_ = get_node()->create_subscription<romCmdType>(
      "~/led1/command",  rclcpp::SystemDefaultsQoS(),
      [this](const romCmdType::SharedPtr msg) { command_ptr1 = msg; });
    subscription_command2_ = get_node()->create_subscription<romCmdType>(
      "~/led2/command",  rclcpp::SystemDefaultsQoS(),
      [this](const romCmdType::SharedPtr msg) { command_ptr2 = msg; });
    subscription_command3_ = get_node()->create_subscription<romCmdType>(
      "~/led3/command",  rclcpp::SystemDefaultsQoS(),
      [this](const romCmdType::SharedPtr msg) { command_ptr3 = msg; });
    subscription_command4_ = get_node()->create_subscription<romCmdType>(
      "~/led4/command",  rclcpp::SystemDefaultsQoS(),
      [this](const romCmdType::SharedPtr msg) { command_ptr4 = msg; });
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::initMsgs()
{ // Edited
  //gpio_msg_.interface_names = inputs_;
  //gpio_msg_.values.resize(inputs_.size());
  estop_msg_.data = 0; 
  gpio_msg1_.data = 0; 
  gpio_msg2_.data = 0; 
  gpio_msg3_.data = 0; 
  gpio_msg4_.data = 0; 

  imu_msg_.header.frame_id = "imu_link"; // this should be equal with urdf and ros2_control xacro
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

controller_interface::CallbackReturn GPIOController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // Edited
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // Edited
  try
  {
    // reset publisher
    stop_publisher0_.reset(); // ?
    gpio_publisher1_.reset();
    gpio_publisher2_.reset();
    gpio_publisher3_.reset();
    gpio_publisher4_.reset();
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
  bobo_hardware::GPIOController, controller_interface::ControllerInterface)
