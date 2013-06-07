#ifndef _MOTION_PRIMITIVE_
#define _MOTION_PRIMITIVE_

#include <vector>
#include <ros/console.h>

namespace sbpl_arm_planner {

typedef std::vector<double> RobotState;

typedef std::vector<RobotState> Action;

typedef struct
{
  int type;
  int id;
  int group;
  Action action;

  void print()
  { 
    ROS_INFO("type: %20s  id: %d  nsteps: %d  group: %d", motion_primitive_type_names_[int(mp.type)].c_str(), mp.id, mp.action.size(), mp.group);
    for(std::size_t j = 0; j < mp.action.size(); ++j)
    {
      ROS_INFO("[step: %d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", j, mp.action[j][0],mp.action[j][1],mp.action[j][2],mp.action[j][3],mp.action[j][4],mp.action[j][5],mp.action[j][6]);
    }
  }

} MotionPrimitive;

}

#endif
