#include <sbpl_arm_planner/collision_checker.h>

CollisionChecker::CollisionChecker()
{
}

bool CollisionChecker::init(std::string name)
{
  group_name_ = name;
  return true;
}

void CollisionChecker::setPlanningJoints(const std::vector<std::string> &planning_joints)
{
  planning_joints_ = planning_joints;
}

bool CollisionChecker::isStateValid(const std::vector<double> &angles, bool verbose, bool visualize, double &dist)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int path_length, int num_checks, double &dist)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::interpolatePath(const std::vector<double> &start, const std::vector<double> &end, const std::vector<double> &inc, std::vector<std::vector<double> > &path)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}


