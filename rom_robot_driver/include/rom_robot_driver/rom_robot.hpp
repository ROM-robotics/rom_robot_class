/**
 * @file include/rom_driver/rom_robot.hpp
 *
 * @brief Device driver core interface.
 *
 * License: BSD
 *   https://raw.github.com/greenghostman/rom_robot_class/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ROM_ROBOT_HPP_
#define ROM_ROBOT_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <iomanip>
#include "wheel.hpp"
#include "rom_serial.hpp"
#include "rom_protocols.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bobo
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/
// struct, union




/*****************************************************************************
 ** Interface [Kobuki]
 *****************************************************************************/
/**
 * @brief  The core kobuki driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class Robot
{
public:
  Robot();
  ~Robot();

  // Delete copy constructor
  Robot(const Robot&) = delete;

    // Delete move constructor
  Robot(Robot&&) = delete;

    // Delete copy assignment operator
  Robot& operator=(const Robot&) = delete;

    // Delete move assignment operator
  Robot& operator=(Robot&&) = delete;

private:
  Wheel left_wheel;
  Wheel right_wheel;
  McuSerial mcu_serial;

  float basewidth = 0;
  
};

} // namespace kobuki

#endif /* ROM_ROBOT_HPP_ */