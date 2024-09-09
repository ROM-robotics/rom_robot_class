#include "bobo_hardware/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

PC_DATA transmit_data = {0,0,0,0};
MCU_DATA receive_data = {0,0,0,0,0,0,0,0};

struct e1234567_status e_leds_status;

//int16_t i=0; int16_t j=0;

//#define ROM_DEBUG

namespace bobo_hardware
{

hardware_interface::CallbackReturn 
Stm32Hardware::on_init(const hardware_interface::HardwareInfo & info)  // DONE
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]); // no use
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stof(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  //cfg_.led_max_volt = std::stoi(info_.hardware_parameters["led_max_volt"]);
  //cfg_.led_min_volt = std::stoi(info_.hardware_parameters["led_min_volt"]);

  cfg_.base_width = std::stof(info_.hardware_parameters["base_width"]);

  RCLCPP_INFO(rclcpp::get_logger(" =================== "), "================================");
  RCLCPP_INFO(rclcpp::get_logger(" \033[1;36mROM DYNAMIC Co.,Ltd \033[1;0m "), " \033[1;36mChecking parameters ... \033[1;0m");
  RCLCPP_INFO(rclcpp::get_logger(" =================== "), "================================");
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stof(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stof(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stof(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stof(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "PID values not supplied, using defaults.");
  }
  
  bobo_robot.left_wheel.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  bobo_robot.right_wheel.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Stm32Hardware"),"Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Stm32Hardware"),"Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Stm32Hardware"),"Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Stm32Hardware"),"Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Stm32Hardware"),"Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ROM ADD
  if (info_.gpios.size() != 5)
  { // AMRSystemWithGPIO မှာ led ၄ ခုနဲ့ estop တခု စုစုပေါင်း ၅ ခုမရှိရင် Error ပြ။
    RCLCPP_FATAL(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "AMRSystemWithGPIO has '%ld' GPIO components, '%d' expected.", info_.gpios.size(), 5);
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < 5; i++)
  { // အဲ့ဒီ gpio တစ်ခုစီမှာ command interface တစ်ခုစီပါရမယ်။
    if (info_.gpios[i].command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "GPIO component %s has '%ld' command interfaces, '%d' expected.", info_.gpios[i].name.c_str(), info_.gpios[i].command_interfaces.size(), 1);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.gpios[i].state_interfaces.size() != 1)
    { // အဲ့ဒီ gpio တစ်ခုစီမှာ state interface တစ်ခုစီပါရမယ်။
      RCLCPP_FATAL(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "GPIO component %s has '%ld' state interfaces, '%d' expected.", info_.gpios[i].name.c_str(), info_.gpios[i].state_interfaces.size(), 1);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ROM END

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
Stm32Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) // Zero assignment လုပ်ပေးထားတယ်။
{
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Configuring ...please wait...");

  // ROM ADD
  std::fill(hw_gpio_commands.begin(), hw_gpio_commands.end(), 0);
  std::fill(hw_gpio_states.begin()  , hw_gpio_states.end()  , 0);

  e_leds_status.estop_status = 0;
  e_leds_status.led1_status  = 0;
  e_leds_status.led2_status  = 0;
  e_leds_status.led3_status  = 0;
  e_leds_status.led4_status  = 0;
  e_leds_status.led5_status  = 0;
  e_leds_status.led6_status  = 0;
  e_leds_status.led7_status  = 0;

  // ROM END
  if (bobo_robot.mcu_serial.connected())
  {
    bobo_robot.mcu_serial.disconnect();
  }
  bobo_robot.mcu_serial.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
Stm32Hardware::export_state_interfaces()     // MCU ကနေ ROS ကို သွားမည့်ကောင်တွေဖြစ်တယ်။
{ 
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(bobo_robot.left_wheel.name, hardware_interface::HW_IF_POSITION, &bobo_robot.left_wheel.actual_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(bobo_robot.left_wheel.name, hardware_interface::HW_IF_VELOCITY, &bobo_robot.left_wheel.actual_velocity_rds));

  state_interfaces.emplace_back(hardware_interface::StateInterface(bobo_robot.right_wheel.name, hardware_interface::HW_IF_POSITION, &bobo_robot.right_wheel.actual_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(bobo_robot.right_wheel.name, hardware_interface::HW_IF_VELOCITY, &bobo_robot.right_wheel.actual_velocity_rds));

  // ROM ADD
 
  hw_gpio_states.resize(4);
  for (size_t i = 0; i < info_.gpios.size(); i++)     // gpio ရှိသလောက်
  {
    for (auto state_if : info_.gpios.at(i).state_interfaces) // gpio တစ်ခုဆီမှာ state interface ရှိသလောက်
    { // gpio ရဲ့ state ကို export လုပ်ရမယ်။
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.gpios.at(i).name, state_if.name, &hw_gpio_states[i]));
      RCLCPP_INFO(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "Added %s/%s",info_.gpios.at(i).name.c_str(), state_if.name.c_str());
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
Stm32Hardware::export_command_interfaces()   // MCU ကို သွားမည့်ကောင်တွေဖြစ်တယ်
{ 
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    bobo_robot.left_wheel.name, hardware_interface::HW_IF_VELOCITY, &bobo_robot.left_wheel.desire_velocity_rds));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    bobo_robot.right_wheel.name, hardware_interface::HW_IF_VELOCITY, &bobo_robot.right_wheel.desire_velocity_rds));

  // ROM ADD
  
  hw_gpio_commands.resize(4);
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto command_if : info_.gpios.at(i).command_interfaces)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.gpios.at(i).name, command_if.name, &hw_gpio_commands[i]));
      RCLCPP_INFO(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "Added %s/%s",info_.gpios.at(i).name.c_str(), command_if.name.c_str());
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn 
Stm32Hardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)   // actual data များ Zero လုပ်ပေးထားတယ်။
{
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Cleaning up ...please wait...");
  if (bobo_robot.mcu_serial.connected())
  {
    bobo_robot.mcu_serial.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn 
Stm32Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)  // desire, actual are zeros
{
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Activating ...please wait...");
  if (!bobo_robot.mcu_serial.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    bobo_robot.mcu_serial.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  
  // command and state should be equal when starting
  // position command , position state သာဆိုရင် ဒီအဆင့်မှာ equal ထားလို့ရတယ်။
  // command and state should be equal when starting
  for (uint i = 0; i < hw_gpio_commands.size(); i++)
  {
    hw_gpio_commands[i] = 0;
    hw_gpio_states[i]   = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
Stm32Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) // ဘာမှမလုပ်
{
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("Stm32Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type 
Stm32Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) // mcu က data ကို actual ထဲထည့်ထားတယ်။
{
  // warning hide
  ROM_UNUSED(period);

  if (!bobo_robot.mcu_serial.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "comms_.connected() is false.");
    return hardware_interface::return_type::ERROR;
  }
  bobo_robot.mcu_serial.read_values(receive_data.right_actual_rpm,receive_data.left_actual_rpm,
                      receive_data.right_encoder_count,receive_data.left_encoder_count,
                      receive_data.volt, receive_data.ampere,receive_data.e1234567,
                      receive_data.checksum);
  int16_t calculated_chksum = (int16_t)(receive_data.right_actual_rpm+receive_data.left_actual_rpm+
                      receive_data.right_encoder_count+receive_data.left_encoder_count+
                      receive_data.volt+receive_data.e1234567);

#ifdef ROM_DEBUG
  //RCLCPP_INFO(rclcpp::get_logger("\033[1;35mTx\033[1;0m"), "\033[1;35m Incoming : %s \033[1;0m", response.c_str());

  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Right Acutal RPM : %hd \033[1;0m",receive_data.right_actual_rpm);
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Left Acutal RPM  : %hd \033[1;0m",receive_data.left_actual_rpm);
  
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Right Acutal Enc : %d \033[1;0m",receive_data.right_encoder_count);
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Left Acutal Enc  : %d \033[1;0m",receive_data.left_encoder_count);

  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Volt             : %hd \033[1;0m",receive_data.volt);
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Ampere           : %.4f \033[1;0m",receive_data.ampere);
  
  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m e1234567         : %hd \033[1;0m",receive_data.e1234567);

  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m CheckSum         : %hd \033[1;0m",receive_data.checksum);

  RCLCPP_INFO(rclcpp::get_logger("\033[1;35mRx\033[1;0m"), "\033[1;35m Calculated cs    : %d \033[1;0m",calculated_chksum);
#endif

  if( receive_data.checksum != calculated_chksum ) 
  { 
    RCLCPP_INFO(rclcpp::get_logger("CHECKSUM"), "\033[1;31mROM checksum calculation mismatch error\033[1;0m");
    
    return hardware_interface::return_type::ERROR; // comment in Production
  }
  else 
  {
    // to transfer leds and estop state to gpio controller
    hw_gpio_states[0] = _Bool(receive_data.e1234567 & 0b0000000010000000);
    hw_gpio_states[1] = _Bool(receive_data.e1234567 & 0b0000000001000000);
    hw_gpio_states[2] = _Bool(receive_data.e1234567 & 0b0000000000100000);
    hw_gpio_states[3] = _Bool(receive_data.e1234567 & 0b0000000000010000);
    hw_gpio_states[4] = _Bool(receive_data.e1234567 & 0b0000000000001000);

    #ifdef ROM_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("\033[1;35mESTOP\033[1;0m"), "\033[1;35m %f \033[1;0m", hw_gpio_states[0]);
    RCLCPP_INFO(rclcpp::get_logger("\033[1;35mLED 1\033[1;0m"), "\033[1;35m %f \033[1;0m", hw_gpio_states[1]);
    RCLCPP_INFO(rclcpp::get_logger("\033[1;35mLED 2\033[1;0m"), "\033[1;35m %f \033[1;0m", hw_gpio_states[2]);
    RCLCPP_INFO(rclcpp::get_logger("\033[1;35mLED 3\033[1;0m"), "\033[1;35m %f \033[1;0m", hw_gpio_states[3]);
    RCLCPP_INFO(rclcpp::get_logger("\033[1;35mLED 4\033[1;0m"), "\033[1;35m %f \033[1;0m", hw_gpio_states[4]);
    RCLCPP_INFO(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "GPIOs hw_gpio_states[] read!");
    #endif

    // Encoder values to wheel positions ( check encoder overflows )
    bobo_robot.left_wheel.encoder_counts  = receive_data.left_encoder_count;
    bobo_robot.right_wheel.encoder_counts = receive_data.right_encoder_count;

    bobo_robot.left_wheel.actual_velocity_rpm  = receive_data.left_actual_rpm;
    bobo_robot.right_wheel.actual_velocity_rpm = receive_data.right_actual_rpm;

    // လိုအပ်ချက်ပေါ်မူတည်ပြီး ပြင်ဆင်ပေးဖို့လိုတယ်။
    bobo_robot.left_wheel.actual_position      = bobo_robot.left_wheel.getActualPosition(receive_data.left_encoder_count);
    bobo_robot.left_wheel.actual_velocity_rds  = bobo_robot.left_wheel.getActualRadianPerSecond();

    bobo_robot.right_wheel.actual_position     = bobo_robot.right_wheel.getActualPosition(receive_data.right_encoder_count);
    bobo_robot.right_wheel.actual_velocity_rds = bobo_robot.right_wheel.getActualRadianPerSecond();

  // RCLCPP_INFO(rclcpp::get_logger("\033[1;36mIncoming Message\033[1;0m"), "\033[1;36m%d %d %d %d %d\033[1;0m", right_wheel.actual_rpm, left_wheel.actual_rpm, right_wheel.enc, left_wheel.enc, right_wheel.bat_status);
  }

  #ifdef ROM_DEBUG
  RCLCPP_INFO(rclcpp::get_logger("ROM AMRSystemWithGPIO"), "GPIOs successfully read!");
  #endif
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type 
Stm32Hardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) // PC က desire ကို e123 ထဲထည့်။
{
  if (!bobo_robot.mcu_serial.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("[ Robot VCOM port ]"), "comms_.connected() is false.");
    return hardware_interface::return_type::ERROR;
  }
  
  // ဒါက radian per second ကို revolute per minute ပြောင်းဖို့လိုတယ်။
  transmit_data.left_desire_rpm  = bobo_robot.left_wheel.getDesireRpm();
  transmit_data.right_desire_rpm = bobo_robot.right_wheel.getDesireRpm();
  // transmit_data.left_desire_rpm  = i++;
  // transmit_data.right_desire_rpm = j++;

  // ROM ADD
  hw_gpio_commands[0] > 0  ?  e_leds_status.estop_status = 1  :  e_leds_status.estop_status = 0;
  hw_gpio_commands[1] > 0  ?  e_leds_status.led1_status  = 1  :  e_leds_status.led1_status  = 0;
  hw_gpio_commands[2] > 0  ?  e_leds_status.led2_status  = 1  :  e_leds_status.led2_status  = 0;
  hw_gpio_commands[3] > 0  ?  e_leds_status.led3_status  = 1  :  e_leds_status.led3_status  = 0;
  hw_gpio_commands[4] > 0  ?  e_leds_status.led4_status  = 1  :  e_leds_status.led4_status  = 0;

  transmit_data.e1234567 = (int16_t) ( (e_leds_status.estop_status<<7) +
  (e_leds_status.led1_status<<6) + (e_leds_status.led2_status<<5) + (e_leds_status.led3_status<<4) +
  (e_leds_status.led4_status<<3) + (e_leds_status.led5_status>>2) + (e_leds_status.led6_status<<1) +
  (e_leds_status.led7_status) );

  
  // ROM END
  transmit_data.checksum = (int16_t)(transmit_data.left_desire_rpm+transmit_data.right_desire_rpm+transmit_data.e1234567);

  bobo_robot.mcu_serial.set_parameter(&transmit_data);

  #ifdef ROM_DEBUG
  //RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m Outgoing : %s \033[1;0m",ss.c_str());

  //RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m rar      : %d \033[1;0m",transmit_data.right_actual_rpm);
  //RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m lar      : %d \033[1;0m",transmit_data.left_actual_rpm);

  //RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m rec      : %d \033[1;0m",transmit_data.right_encoder_count);
  //RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m lec      : %d \033[1;0m",transmit_data.left_encoder_count);

  RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m rdr, ldr : %d %d \033[1;0m", transmit_data.right_desire_rpm, transmit_data.left_desire_rpm);
  
  // RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m reserve  : %d \033[1;0m",transmit_data.reserve);
  RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m LED      : %d \033[1;0m",transmit_data.e1234567);

  // RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m Volt     : %d \033[1;0m",transmit_data.volt);
  // RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m Ampere   : %.2f \033[1;0m",transmit_data.ampere);

  RCLCPP_INFO(rclcpp::get_logger("\033[1;32mTx\033[1;0m"), "\033[1;32m CheckSum : %d \033[1;0m",transmit_data.checksum);
  #endif
  
  return hardware_interface::return_type::OK;
}

}  // namespace bobo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  bobo_hardware::Stm32Hardware, hardware_interface::SystemInterface)