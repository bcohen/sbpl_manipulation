#include <ros/ros.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_collision_space_test");
  ros::Publisher p = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500, true);
  ros::NodeHandle ph("~");
  sleep(1);
  std::string group_name, world_frame;
  std::vector<double> dims(3, 0.0), origin(3, 0.0);
  std::vector<std::string> joint_names(7);
  ph.param<std::string>("group_name", group_name, "");
  ph.param<std::string>("world_frame", world_frame, "");
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

  distance_field::PropagationDistanceField *df = new distance_field::PropagationDistanceField(dims[0], dims[1], dims[2], 0.02, origin[0], origin[1], origin[2], 0.4);
  df->reset();

  sbpl_arm_planner::OccupancyGrid* grid = new sbpl_arm_planner::OccupancyGrid(df);
  grid->setReferenceFrame(world_frame);

  sbpl_arm_planner::CollisionChecker* cspace = new sbpl_arm_planner::SBPLCollisionSpace(grid);

  if(!cspace->init(group_name))
    return false;
  ROS_INFO("Initialized the collision space.");

  if(!cspace->setPlanningJoints(joint_names))
    return false;

  // add robot's pose in map
  arm_navigation_msgs::PlanningScenePtr scene(new arm_navigation_msgs::PlanningScene);
  scene->collision_map.header.frame_id = world_frame;
  scene->robot_state.multi_dof_joint_state.frame_ids.resize(2);
  scene->robot_state.multi_dof_joint_state.child_frame_ids.resize(2);
  scene->robot_state.multi_dof_joint_state.poses.resize(2);
  scene->robot_state.multi_dof_joint_state.frame_ids[0] = "base_link";
  scene->robot_state.multi_dof_joint_state.child_frame_ids[0] = "torso_lift_link";
  scene->robot_state.multi_dof_joint_state.poses[0].position.x = -0.06;
  scene->robot_state.multi_dof_joint_state.poses[0].position.y = 0.0;
  scene->robot_state.multi_dof_joint_state.poses[0].position.z = 0.34;
  scene->robot_state.multi_dof_joint_state.poses[0].orientation.w = 1;
  scene->robot_state.multi_dof_joint_state.frame_ids[1] = "base";
  scene->robot_state.multi_dof_joint_state.child_frame_ids[1] = "torso";
  scene->robot_state.multi_dof_joint_state.poses[1].position.x = 0;
  scene->robot_state.multi_dof_joint_state.poses[1].position.y = 0;
  scene->robot_state.multi_dof_joint_state.poses[1].position.z = 0;
  scene->robot_state.multi_dof_joint_state.poses[1].orientation.w = 1;

  scene->robot_state.joint_state.name.push_back("right_gripper_finger_joint");
  scene->robot_state.joint_state.name.push_back("left_gripper_finger_joint");
  scene->robot_state.joint_state.position.push_back(0.08);
  scene->robot_state.joint_state.position.push_back(0.08);
  cspace->setPlanningScene(*scene);

  std::vector<double> angles(7,0);
  angles[0] = -0.7;
  angles[1] = 0.3;
  angles[2] = 0.0;
  angles[3] = 0.5;
  angles[4] = 0.6;
  angles[5] = 0.8;
  angles[6] = 0.4;

  ros::spinOnce();
  p.publish(cspace->getVisualization("distance_field"));
  p.publish(cspace->getVisualization("bounds"));
  //ma_pub.publish(planner->getVisualization("bfs_walls"));
  //ma_pub.publish(planner->getVisualization("bfs_values"));
  //ma_pub.publish(planner->getVisualization("goal"));
  //ma_pub.publish(planner->getVisualization("expansions"));
  p.publish(cspace->getVisualization("collision_objects"));
  p.publish(cspace->getVisualization("occupied_voxels"));
  p.publish(cspace->getCollisionModelVisualization(angles));
  //p.publish(cspace->getMeshModelVisualization("arm", angles));

  ros::spinOnce();
  sleep(1);

  ROS_INFO("Done");
  delete cspace;
  delete grid;
  delete df;
  return 0;
}

