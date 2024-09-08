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

#ifndef ROM_ROBOT_CLASS_HPP_
#define ROM_ROBOT_CLASS_HPP_

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

namespace rom_dynamics
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/
// struct, union




/*****************************************************************************
 ** Interface [bobo]
 *****************************************************************************/
/**
 * @brief  The core bobo driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
  class Robot
  {
    public:
      Robot()
      {

      }
      ~Robot();

      Robot(const Robot&) = delete;
      Robot(Robot&&) = delete;
      Robot& operator=(const Robot&) = delete;
      Robot& operator=(Robot&&) = delete;

      /* id */
      std::string robot_name;
      short robot_id;
      
      /* mobile base */
      Wheel left_wheel;
      Wheel right_wheel;
      float basewidth;

      /* serial */
      McuSerial mcu_serial;

      /* serialize, deserialize */

      /* sensors */

  };

} // namespace bobo

#endif /* ROM_ROBOT_CLASS_HPP_ */