#include <ros/ros.h>
#include <sbpl_collision_checking/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_collision_checking/sbpl_collision_interface.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_collision_space");
  unsigned char dist = 0;
  ros::Publisher p = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500, true);
  sleep(1);

  std::vector<std::string> names(7);
  names[1] = "joint2_shoulder_pitch";
  names[5] = "joint6_wrist_pitch";
  names[2] = "joint3_arm_roll";
  names[0] = "joint1_shoulder_yaw";
  names[6] = "joint7_wrist_roll";
  names[4] = "joint5_forearm_roll";
  names[3] = "joint4_elbow";

  sbpl_arm_planner::OccupancyGrid* grid = new sbpl_arm_planner::OccupancyGrid(2.0, 2.0, 2.0, 0.02, -0.6, -1.15, -0.1);

  sbpl_arm_planner::SBPLCollisionSpace* cspace = new sbpl_arm_planner::SBPLCollisionSpace(grid);

  if(!cspace->init("hdt_arm"))
    return false;

  if(!cspace->setPlanningJoints(names))
    return false;

  cspace->setPadding(0.005);


  std::vector<double> angles(7,0);
  angles[0] = 0.7;
  angles[1] = 0.3;
  angles[2] = 0.0;
  angles[3] = 0.5;
  angles[4] = 0.6;
  angles[5] = 0.8;
  angles[6] = 0.4;

  bool res = cspace->checkCollision(angles, true, false, dist);

  ROS_ERROR("angles: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f  valid? %d  dist: %d", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], res, int(dist));


  sbpl_arm_planner::SBPLCollisionInterface* cface = new sbpl_arm_planner::SBPLCollisionInterface(grid, cspace);
  if(!cface->init())
    return false;

  visualization_msgs::MarkerArray m;
  std_msgs::ColorRGBA color;
  color.r = 1;
  color.a = 1;
  cface->getCollisionModelVisualization(angles, "hdt", 0, color, m.markers);
  p.publish(m);
  ros::spin();
  sleep(0.5);
  ros::spin();
  p.publish(m);
  ros::spin();


  /*
  if(!model.init())
    ROS_ERROR("Model failed to initialize.");

  if(!model.initGroup("hdt_arm"))
    ROS_ERROR("Model failed to initialize group.");

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

  ROS_INFO("[test] Setting the order of the joint positions now.");
  model.setOrderOfJointPositions(names, "hdt_arm");
  model.setDefaultGroup("hdt_arm");
  //model.printDebugInfo("hdt_arm");

  
  std::vector<std::vector<KDL::Frame> > frames;
  if(!model.computeDefaultGroupFK(angles, frames))
    ROS_ERROR("FK Solver failed");

  model.printFrames(frames);  
  //model.printGroups();
  ROS_INFO("\n\nexiting.");
  */

  ros::spin();
  return 0;
}

