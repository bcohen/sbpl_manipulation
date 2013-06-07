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

enum MotionPrimitiveType {
  LONG_DISTANCE,
  SHORT_DISTANCE,
  ADAPTIVE,
  SNAP_TO_RPY,
  SNAP_TO_XYZRPY,
  SNAP_TO_RPY_THEN_TO_XYZ,
  SNAP_TO_XYZ_THEN_TO_RPY,
  SNAP_TO_RPY_AT_START,
  RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ,
  RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ,
  NUMBER_OF_MPRIM_TYPES
};

class ActionSet
{
  public:
    ActionSet();

    ~ActionSet(){};

    bool init(FILE* fCfg, EnvironmentROBARM3D &env);

    bool getMotionPrimitivesFromFile(FILE* fCfg);

    void addMotionPrim(std::vector<double> mprim, bool add_converse, bool short_dist_mprim);

    void print();

    bool getActionSet(const std::vector<double> &parent, std::vector<MotionPrimitive> &actions);

  private:

    bool use_multires_mprims_;

    bool use_ik_;

    double short_dist_mprims_thresh_m_;

    double solve_for_ik_thresh_m_;

    std::vector<MotionPrimitive> mp_;

    std::vector<std::string> motion_primitive_type_names_;
};

}
#endif

