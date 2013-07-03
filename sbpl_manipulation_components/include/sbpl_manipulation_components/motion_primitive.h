#ifndef _MOTION_PRIMITIVE_
#define _MOTION_PRIMITIVE_

#include <vector>
#include <sstream>
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
    std::stringstream os;
    for(std::size_t j = 0; j < action.size(); ++j)
    {
      os.str("");
      os << "[step: " << int(j+1) << "/" << int(action.size()) << "] ";
      for(std::size_t k = 0; k < action[j].size(); ++k)
        os << std::setw(4) << std::setprecision(3) << std::fixed << action[j][k] << " ";
      ROS_INFO_STREAM(os.str());
    }
  }

} MotionPrimitive;

}

#endif
