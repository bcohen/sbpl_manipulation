#ifndef _MOTION_PRIMITIVE_
#define _MOTION_PRIMITIVE_

#include <vector>
#include <ros/console.h>

namespace sbpl_arm_planner {

typedef std::vector<double> RobotState;

typedef std::vector<RobotState> Action;

typedef struct MotionPrimitive
{
  int type;
  int id;
  int group;
  Action action;

  void print()
  { 
    ROS_INFO("type: %d  id: %d  nsteps: %d  group: %d", type, id, int(action.size()), group);
    for(std::size_t j = 0; j < action.size(); ++j)
    {
      ROS_INFO("[step: %d/%d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", int(j+1),int(action.size()), action[j][0],action[j][1],action[j][2],action[j][3],action[j][4],action[j][5],action[j][6]);
    }
  }

} MotionPrimitive;

}

#endif
