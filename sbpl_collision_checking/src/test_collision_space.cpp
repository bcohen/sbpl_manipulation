#include <ros/ros.h>
#include <sbpl_collision_checking/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_collision_space_test");
  double dist = 0;
  ros::Publisher p = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500, true);
  ros::NodeHandle ph("~");
  sleep(1);
  std::string group_name;
  std::vector<double> dims(3, 0.0), origin(3, 0.0);
  std::vector<std::string> joint_names(7);
  ph.param<std::string>("group_name", group_name, "");
  ph.param("dims/x", dims[0], 2.0);
  ph.param("dims/y", dims[1], 2.0);
  ph.param("dims/z", dims[2], 2.0);
  ph.param("origin/x", origin[0], -0.75);
  ph.param("origin/y", origin[1], 1.25);
  ph.param("origin/z", origin[2], -0.3);
  ph.param<std::string>("joint_0", joint_names[0], "");
  ph.param<std::string>("joint_1", joint_names[1], "");
  ph.param<std::string>("joint_2", joint_names[2], "");
  ph.param<std::string>("joint_3", joint_names[3], "");
  ph.param<std::string>("joint_4", joint_names[4], "");
  ph.param<std::string>("joint_5", joint_names[5], "");
  ph.param<std::string>("joint_6", joint_names[6], "");

  // remove empty joint names (for arms with fewer than 7 joints)
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    if(joint_names[i].empty())
      joint_names.erase(joint_names.begin()+i);
  }
  if(joint_names.empty())
  {
    ROS_ERROR("No planning joints found on param server.");
    return 0;
  }
  ROS_INFO("Retrieved %d planning joints from param server.", int(joint_names.size()));

  sbpl_arm_planner::OccupancyGrid* grid = new sbpl_arm_planner::OccupancyGrid(dims[0], dims[1], dims[2], 0.02, origin[0], origin[1], origin[2]);
  grid->setReferenceFrame("base_footprint");

  sbpl_arm_planner::SBPLCollisionSpace* cspace = new sbpl_arm_planner::SBPLCollisionSpace(grid);

  if(!cspace->init(group_name))
    return false;
  ROS_INFO("Initialized the collision space.");

  if(!cspace->setPlanningJoints(joint_names))
    return false;

  cspace->setPadding(0.005);

  grid->setReferenceFrame("base_footprint");

  std::vector<double> angles(7,0);
  angles[0] = 0.7;
  angles[1] = 0.3;
  angles[2] = 0.0;
  angles[3] = 0.5;
  angles[4] = 0.6;
  angles[5] = 0.8;
  angles[6] = 0.4;

  //bool res = cspace->checkCollision(angles, true, false, dist);

  //ROS_ERROR("angles: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f  valid? %d  dist: %d", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], res, int(dist));

  // visualize the collision model
  visualization_msgs::MarkerArray m;
  m = cspace->getVisualization("bounds");
  p.publish(m);
  ros::spinOnce();
  m = cspace->getVisualization("distance_field");
  p.publish(m);
  ros::spinOnce();
  m = cspace->getCollisionModelVisualization(angles);
  p.publish(m);
  ros::spinOnce();
  sleep(0.5);
  p.publish(m);
  ros::spinOnce();
  
  ros::spin();
  ROS_INFO("Done");
  delete cspace;
  delete grid;
  return 0;
}

