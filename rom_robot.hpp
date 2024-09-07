/**
 * @file include/rom_driver/rom_robot.hpp
 *
 * @brief Device driver core interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
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


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/
// struct, union


/*****************************************************************************
** Parent Interface
*****************************************************************************/

class PacketFinder : public PacketFinderBase
{
public:
  virtual ~PacketFinder() {}
  bool checkSum();
};

/*****************************************************************************
 ** Interface [Kobuki]
 *****************************************************************************/
/**
 * @brief  The core kobuki driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class Kobuki
{
public:
  Kobuki();
  ~Kobuki();

  /*********************
   ** Configuration
   **********************/


  /******************************************
  ** Packet Processing
  *******************************************/


  /******************************************
  ** Getters - Data Protection
  *******************************************/


  /******************************************
  ** Getters - User Friendly Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */


  /******************************************
  ** Getters - Raw Data Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */


  /*********************
  ** Feedback
  **********************/


  /*********************
  ** Soft Commands
  **********************/
  void resetOdometry();

  /*********************
  ** Hard Commands
  **********************/


  /*********************
  ** Debugging
  **********************/


private:
  /*********************
  ** Thread
  **********************/


  /*********************
  ** Odometry
  **********************/


  /*********************
  ** Inertia
  **********************/


  /*********************
  ** Driver Paramters
  **********************/


  /*********************
  ** Acceleration Limiter
  **********************/


  /*********************
  ** Packet Handling
  **********************/


  /*********************
  ** Commands
  **********************/


  /*********************
  ** Events
  **********************/

  /*********************
  ** Logging
  **********************/


  /*********************
  ** Signals
  **********************/

};

} // namespace kobuki

#endif /* ROM_ROBOT_HPP_ */