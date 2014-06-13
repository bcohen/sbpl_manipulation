#include <sbpl_manipulation_components/collision_checker.h>

namespace sbpl_arm_planner {

CollisionChecker::CollisionChecker()
{
}

bool CollisionChecker::init(std::string name, std::string ns)
{
  group_name_ = name;
  return true;
}

bool CollisionChecker::setPlanningJoints(const std::vector<std::string> &planning_joints)
{
  planning_joints_ = planning_joints;
  return true;
}

void CollisionChecker::setRobotState(const arm_navigation_msgs::RobotState &state)
{
  robot_state_ = state;
}

bool CollisionChecker::setAttachedObjects(const std::vector<arm_navigation_msgs::AttachedCollisionObject> &objects)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::setPlanningScene(const arm_navigation_msgs::PlanningScene &scene)
{
  planning_scene_  = scene;
  return false;
}

bool CollisionChecker::isStateValid(const std::vector<double> &angles, bool verbose, bool visualize, double &dist)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::isStateValid(const std::vector<double> &angles, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, bool verbose, bool visualize, double &dist)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path)

{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::interpolatePath(const std::vector<double> &start, const std::vector<double> &end, const std::vector<double> &inc, std::vector<std::vector<double> > &path)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

visualization_msgs::MarkerArray CollisionChecker::getCollisionModelVisualization(const std::vector<double> &angles)
{
  ROS_ERROR("Function is not filled in.");
  return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray CollisionChecker::getVisualization(std::string type)
{
  ROS_ERROR("Function is not filled in.");
  return visualization_msgs::MarkerArray();
}

void CollisionChecker::setSphereGroupsForCollisionCheck(const std::vector<std::string> &group_names)
{
  ROS_ERROR("Function is not filled in.");
  return;
}

bool CollisionChecker::isObjectAttached()
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

}
