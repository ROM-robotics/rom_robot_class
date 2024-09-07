#ifndef stm32f4_system_interface_gpio_COMMS_HPP
#define stm32f4_system_interface_gpio_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cstdint>

#include "rom_protocol.h"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class STM32Board
{

public:

  STM32Board() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  std::string read_msg()
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }
    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_values(int16_t &r_act_rpm, int16_t &l_act_rpm, int32_t &r_enc, int32_t &l_enc, int16_t &volt, float &amp, int16_t &led, int16_t &checksum)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }
    
    std::istringstream iss(response);
    
    std::vector<std::string> tokens;
    std::string token;
    
    while( std::getline( iss, token, ' ') ) 
    {
    	tokens.push_back(token);
    }
    
    r_act_rpm = std::atoi(tokens[0].c_str());
    l_act_rpm = std::atoi(tokens[1].c_str());
    r_enc     = std::atoi(tokens[2].c_str());
    l_enc     = std::atoi(tokens[3].c_str());
    volt      = std::atoi(tokens[4].c_str());
    amp       = std::atof(tokens[5].c_str());
    led       = std::atoi(tokens[6].c_str());
    checksum  = std::atoi(tokens[7].c_str());
  }

  // std::string read_stm32f4()
  // {
  //   std::string response = read_msg();
  //   return response;
  // }
  
  void set_parameter(PC_DATA *my_struct)
  {
    std::stringstream ss;

    // ss << my_struct->right_desire_rpm << " "  << my_struct->left_desire_rpm << " " 
    //    << (int16_t)(my_struct->reserve) << " "  << (int16_t)(my_struct->e1234567) << " "  << (int16_t)(my_struct->volt) << " " 
    //    << my_struct->ampere << " "  << (int16_t)(my_struct->checksum) << "\r\n";
    ss << my_struct->right_desire_rpm << " "  << my_struct->left_desire_rpm << " " 
        << " "  << (my_struct->e1234567) << " " << (int16_t)(my_struct->checksum) << "\r\n";

    send_msg(ss.str());
  }

  void set_motor_values(int right_wheel_rpm, int left_wheel_rpm, int bat_status)
  {
    std::stringstream ss;
    ss << right_wheel_rpm << " " << left_wheel_rpm << " " << bat_status << "\r\n";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r\n";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // stm32f4_system_interface_gpio_COMMS_HPP