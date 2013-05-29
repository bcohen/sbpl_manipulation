#include <ros/ros.h>
#include <sbpl_collision_checking/sbpl_collision_model.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_collision_model");
  sbpl_arm_planner::SBPLCollisionModel model;

  if(!model.init())
    ROS_ERROR("Model failed to initialize.");

  if(!model.initGroup("hdt_arm"))
    ROS_ERROR("Model failed to initialize group.");

//  model.printGroups();

  std::vector<double> min_limits, max_limits;
  std::vector<bool> continuous;
  std::vector<std::string> joint_names;

  if(!model.getJointLimits("base_link", "link7", joint_names, min_limits, max_limits, continuous))
    return false;

  // group names
  std::vector<std::string> group_names;
  model.getGroupNames(group_names);
  ROS_INFO("[test] number of groups: %d", int(group_names.size()));
  for(size_t i = 0; i < group_names.size(); ++i)
    ROS_INFO("[test] [%d] group_name: %s", int(i), group_names[i].c_str());

  // initialize groups
  /*
  if(!model.initAllGroups())
    ROS_ERROR("Failed to initialize the collision groups.");
  */

  std::vector<std::string> names(7);
  names[0] = "joint2_shoulder_pitch";
  names[1] = "joint6_wrist_pitch";
  names[2] = "joint3_arm_roll";
  names[3] =  "joint1_shoulder_yaw";
  names[4] = "joint7_wrist_roll";
  names[5] = "joint5_forearm_roll";
  names[6] = "joint4_elbow";

  ROS_INFO("[test] Setting the order of the joint positions now.");
  model.setOrderOfJointPositions(names, "hdt_arm");
  model.setDefaultGroup("hdt_arm");
  //model.printDebugInfo("hdt_arm");

  std::vector<double> angles(7,0);
  angles[0] = 0.7;
  angles[1] = 0.3;
  angles[2] = 0.0;
  angles[3] = 0.5;
  angles[4] = 0.6;
  angles[5] = 0.8;
  angles[6] = 0.4;

  std::vector<std::vector<KDL::Frame> > frames;
  if(!model.computeDefaultGroupFK(angles, frames))
    ROS_ERROR("FK Solver failed");

  model.printFrames(frames);  
  //model.printGroups();
  ROS_INFO("\n\nexiting.");
  return 0;
}

