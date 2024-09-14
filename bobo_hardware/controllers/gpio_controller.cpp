#include "bobo_hardware/gpio_controller.hpp"

#include <string>

#define ROM_DEBUG 1
namespace bobo_hardware
{
controller_interface::CallbackReturn GPIOController::on_init()
{ // OK
  try
  {
    auto_declare<std::vector<std::string>>("inputs", std::vector<std::string>()); // param ရဲ့ inputs
    auto_declare<std::vector<std::string>>("outputs", std::vector<std::string>());// param ရဲ့ outputs
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{ // config လုပ်ပြီးသားကို return ပေးတယ်။
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // ALL, INDIVIDUAL, NONE
  config.names = outputs_; // param ထဲက output string တွေသည် IfConfiguration ထဲရောက်သွားမယ်။

  return config;
}

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
{ // config လုပ်ပြီးသားကို return ပေးတယ်။
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // ALL, INDIVIDUAL, NONE
  config.names = inputs_; // param ထဲက output string တွေသည် IfConfiguration ထဲရောက်သွားမယ်။

  return config;
}

controller_interface::return_type GPIOController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{ // ROM  edit 

  //RCLCPP_DEBUG(
      // get_node()->get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "%s: (%f)", state_interfaces_[0].get_name().c_str(),
      // state_interfaces_[0].get_value());
    #ifdef ROM_DEBUG
      RCLCPP_INFO(rclcpp::get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "\033 %s (%f)\033[1;0m",state_interfaces_[0].get_name().c_str(),
      state_interfaces_[0].get_value());
       RCLCPP_INFO(rclcpp::get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "\033 %s (%f)\033[1;0m",state_interfaces_[1].get_name().c_str(),
      state_interfaces_[1].get_value());
       RCLCPP_INFO(rclcpp::get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "\033 %s (%f)\033[1;0m",state_interfaces_[2].get_name().c_str(),
      state_interfaces_[2].get_value());
       RCLCPP_INFO(rclcpp::get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "\033 %s (%f)\033[1;0m",state_interfaces_[3].get_name().c_str(),
      state_interfaces_[3].get_value());
       RCLCPP_INFO(rclcpp::get_logger("\033[1;35mROM DYNAMICS\033[1;0m"), "\033 %s (%f)\033[1;0m",state_interfaces_[4].get_name().c_str(),
      state_interfaces_[4].get_value());
    #endif

  estop_msg_.data = static_cast<float>(state_interfaces_[0].get_value());
  stop_publisher0_->publish(estop_msg_);
  
  gpio_msg1_.data = static_cast<float>(state_interfaces_[1].get_value());
  gpio_publisher1_->publish(gpio_msg1_);

  gpio_msg2_.data = static_cast<float>(state_interfaces_[2].get_value());
  gpio_publisher2_->publish(gpio_msg2_);

  gpio_msg3_.data = static_cast<float>(state_interfaces_[3].get_value());
  gpio_publisher3_->publish(gpio_msg3_);

  gpio_msg4_.data = static_cast<float>(state_interfaces_[4].get_value());
  gpio_publisher4_->publish(gpio_msg4_);

  //add something 

  if (command_ptr0 != nullptr) {
    command_interfaces_[0].set_value(command_ptr0->data);
  }
  if (command_ptr1 != nullptr) {
    command_interfaces_[1].set_value(command_ptr1->data);
    #ifdef ROM_DEBUG
    RCLCPP_INFO(get_node()->get_logger(), "%s: (%f)", command_interfaces_[1].get_name().c_str(), command_interfaces_[1].get_value());
    #endif
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
    inputs_ = get_node()->get_parameter("inputs").as_string_array();   // input paramter ကိုရယူတယ်။
    outputs_ = get_node()->get_parameter("outputs").as_string_array(); // output parameter ကို ရယူတယ်။

    initMsgs();

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
{ 
  estop_msg_.data = 0; 
  gpio_msg1_.data = 0; 
  gpio_msg2_.data = 0; 
  gpio_msg3_.data = 0; 
  gpio_msg4_.data = 0; 
}

controller_interface::CallbackReturn GPIOController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // Edited
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // OK
  try
  {
    // reset publisher
    stop_publisher0_.reset(); 
    gpio_publisher1_.reset();
    gpio_publisher2_.reset();
    gpio_publisher3_.reset();
    gpio_publisher4_.reset();
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
