#ifndef _ACTION_SET_
#define _ACTION_SET_

#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <ros/ros.h>
#include <angles/angles.h>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <sbpl_manipulation_components/motion_primitive.h>

namespace sbpl_arm_planner {

class EnvironmentROBARM3D;

namespace MotionPrimitiveType {
enum {
  LONG_DISTANCE,
  SHORT_DISTANCE,
  SNAP_TO_RPY,
  SNAP_TO_XYZ_RPY,
  NUMBER_OF_MPRIM_TYPES
};
}

class ActionSet
{
  public:
    ActionSet(std::string action_file);

    ~ActionSet(){};

    bool init(EnvironmentROBARM3D *env);

    bool getActionSet(const RobotState &parent, std::vector<Action> &actions);

    void print();

  protected:

    bool use_multires_mprims_;

    bool use_ik_;

    double short_dist_mprims_thresh_m_;

    double ik_amp_dist_thresh_m_;

    std::string action_file_;

    EnvironmentROBARM3D *env_;

    std::vector<MotionPrimitive> mp_;

    std::vector<std::string> motion_primitive_type_names_;

    bool getMotionPrimitivesFromFile(FILE* fCfg);

    void addMotionPrim(const std::vector<double> &mprim, bool add_converse, bool short_dist_mprim);

    bool applyMotionPrimitive(const RobotState &state, MotionPrimitive &mp, Action &action);

    bool getAction(const RobotState &parent, double dist_to_goal, MotionPrimitive &mp, Action &action);
};

}
#endif

