#include <ros/ros.h>
#include <leatherman/utils.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <visualization_msgs/MarkerArray.h>

#define VISUALIZE false
#define DEBUG false
#define NUM_TRIALS 5
#define NUM_TIMING_SAMPLES 100000

std::vector<double> getRandomJointPositions(const std::vector<double> &min_limits, const std::vector<double> &max_limits, const std::vector<bool> &continuous)
{
  double r;
  std::vector<double> angles(min_limits.size(),0.0);

  for(size_t i = 0; i < min_limits.size(); ++i)
  {
    r = double(rand())/double(RAND_MAX);
    angles[i] = min_limits[i] + r * (max_limits[i] - min_limits[i]);
  }
  return angles;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_sbpl_collision_space_test");
  srand(time(NULL));
  double dist = 0;
  ros::Publisher p = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500, true);
  ros::NodeHandle nh, ph("~");
  sleep(1);
  std::string group_name, world_frame, robot_description;
  std::vector<double> dims(3, 0.0), origin(3, 0.0);
  std::vector<std::string> r_joint_names, l_joint_names, r_finger_names, l_finger_names;
  std::vector<double> r_min_limits, r_max_limits, l_min_limits, l_max_limits;
  std::vector<bool> r_continuous, l_continuous;
  ph.param<std::string>("group_name", group_name, "");
  ph.param<std::string>("world_frame", world_frame, "");
  ph.param("dims/x", dims[0], 3.0);
  ph.param("dims/y", dims[1], 3.0);
  ph.param("dims/z", dims[2], 2.0);
  ph.param("origin/x", origin[0], -0.9);
  ph.param("origin/y", origin[1], 1.25);
  ph.param("origin/z", origin[2], -0.3);
  r_joint_names.push_back("r_shoulder_pan_joint");
  r_joint_names.push_back("r_shoulder_lift_joint");
  r_joint_names.push_back("r_upper_arm_roll_joint");
  r_joint_names.push_back("r_elbow_flex_joint");
  r_joint_names.push_back("r_forearm_roll_joint");
  r_joint_names.push_back("r_wrist_flex_joint");
  r_joint_names.push_back("r_wrist_roll_joint");
  r_finger_names.push_back("r_gripper_r_finger_joint");
  r_finger_names.push_back("r_gripper_l_finger_joint");
  r_finger_names.push_back("r_gripper_r_finger_tip_joint");
  r_finger_names.push_back("r_gripper_l_finger_tip_joint");
  l_joint_names.push_back("l_shoulder_pan_joint");
  l_joint_names.push_back("l_shoulder_lift_joint");
  l_joint_names.push_back("l_upper_arm_roll_joint");
  l_joint_names.push_back("l_elbow_flex_joint");
  l_joint_names.push_back("l_forearm_roll_joint");
  l_joint_names.push_back("l_wrist_flex_joint");
  l_joint_names.push_back("l_wrist_roll_joint");
  l_finger_names.push_back("l_gripper_r_finger_joint");
  l_finger_names.push_back("l_gripper_l_finger_joint");
  l_finger_names.push_back("l_gripper_r_finger_tip_joint");
  l_finger_names.push_back("l_gripper_l_finger_tip_joint");


  distance_field::PropagationDistanceField *df = new distance_field::PropagationDistanceField(dims[0], dims[1], dims[2], 0.02, origin[0], origin[1], origin[2], 0.4);
  df->reset();

  sbpl_arm_planner::OccupancyGrid* grid = new sbpl_arm_planner::OccupancyGrid(df);
  grid->setReferenceFrame(world_frame);

  sbpl_arm_planner::SBPLCollisionSpace* cspace = new sbpl_arm_planner::SBPLCollisionSpace(grid);

  if(!cspace->init(group_name))
    return false;
  ROS_INFO("Initialized the collision space.");

  if(!cspace->setPlanningJoints(r_joint_names))
    return false;

  // get joint limits
  boost::shared_ptr<urdf::Model> urdf;
  if (nh.getParam("robot_description", robot_description))
  {
    urdf = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!urdf->initString(robot_description))
    {
      ROS_WARN("Failed to parse the URDF");
      return 0;
    }
  }
  if(!leatherman::getJointLimits(urdf.get(), "torso_lift_link", "r_gripper_palm_link", r_joint_names, r_min_limits, r_max_limits, r_continuous, false) ||
     !leatherman::getJointLimits(urdf.get(), "torso_lift_link", "l_gripper_palm_link", l_joint_names, l_min_limits, l_max_limits, l_continuous, false))
  {
    ROS_ERROR("Failed to get joint limits.");
    return 0;
  }

  // add robot's pose in map
  arm_navigation_msgs::PlanningScenePtr scene(new arm_navigation_msgs::PlanningScene);
  scene->collision_map.header.frame_id = world_frame;
  scene->robot_state.multi_dof_joint_state.frame_ids.resize(2);
  scene->robot_state.multi_dof_joint_state.child_frame_ids.resize(2);
  scene->robot_state.multi_dof_joint_state.poses.resize(2);
  scene->robot_state.multi_dof_joint_state.frame_ids[0] = "base_footprint";
  scene->robot_state.multi_dof_joint_state.child_frame_ids[0] = "torso_lift_link";
  scene->robot_state.multi_dof_joint_state.poses[0].position.x = -0.05;
  scene->robot_state.multi_dof_joint_state.poses[0].position.y = 0.0;
  scene->robot_state.multi_dof_joint_state.poses[0].position.z = 0.74;
  scene->robot_state.multi_dof_joint_state.poses[0].orientation.w = 1;

  scene->robot_state.joint_state.name.push_back("torso_lift_joint");
  scene->robot_state.joint_state.position.push_back(0.29);
  cspace->setPlanningScene(*scene);

  ros::spinOnce();
  p.publish(cspace->getVisualization("distance_field"));
  p.publish(cspace->getVisualization("bounds"));
  p.publish(cspace->getVisualization("collision_objects"));

  // prepare planning scene for non-planning arm
  arm_navigation_msgs::RobotState robot_state;
  for(size_t j = 0; j < l_joint_names.size(); ++j)
    robot_state.joint_state.name.push_back(l_joint_names[j]);
  robot_state.joint_state.position.resize(l_joint_names.size());

  std::vector<double> langles, rangles, trial_time, ccps;
  for(size_t k = 0; k < NUM_TRIALS; ++k)
  {
    int invalid = 0, valid = 0;
    clock_t start = clock();
    for(size_t i = 0; i < NUM_TIMING_SAMPLES; ++i)
    {
      langles = getRandomJointPositions(l_min_limits, l_max_limits, l_continuous);
      rangles = getRandomJointPositions(r_min_limits, r_max_limits, r_continuous);

      for(size_t j = 0; j < langles.size(); ++j)
        robot_state.joint_state.position[j] = langles[j];
      cspace->setRobotState(robot_state);

      if(DEBUG)
      {
        ROS_INFO("[ left] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
        ROS_INFO("[right] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
      }
      if(!cspace->isStateValid(rangles, false, false, dist))
        invalid++;
      else
        valid++;

      if(VISUALIZE)
      {
        
      }
    }
    trial_time.push_back((clock()-start)/(double)CLOCKS_PER_SEC);
    ccps.push_back(double(NUM_TIMING_SAMPLES)/trial_time.back());
    ROS_INFO("[%d] Collision checked %d samples in %0.5f seconds.  (valid: %d  invalid: %d  checks_per_sec: %0.1f)", int(k), NUM_TIMING_SAMPLES, trial_time.back(), valid, invalid, ccps.back());
  }

  ros::spinOnce();
  sleep(1);

  ROS_INFO("Done");
  delete cspace;
  delete grid;
  delete df;
  return 0;
}

