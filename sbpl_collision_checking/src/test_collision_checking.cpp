#include <ros/ros.h>
#include <leatherman/utils.h>
#include <leatherman/viz.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <visualization_msgs/MarkerArray.h>
#include <pviz/pviz.h>

#define DEBUG false
#define NUM_TRIALS 1
#define NUM_TIMING_SAMPLES 1000000


bool getCollisionObjects(std::string filename, std::vector<arm_navigation_msgs::CollisionObject> &collision_objects, visualization_msgs::MarkerArray &ma)
{
  int num_obs;
  char sTemp[1024];
  visualization_msgs::Marker marker;
  std::vector<std::vector<double> > objects, object_colors;
  std::vector<std::string> object_ids;
  arm_navigation_msgs::CollisionObject object;
  collision_objects.clear();

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    ROS_INFO("Parsed string has length < 1.(number of obstacles)\n");
  ROS_DEBUG("Parsing collision object file with %i objects.",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs, std::vector<double>(6,0.0));
  object_colors.resize(num_obs, std::vector<double>(4,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1. (object parameters for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
    for(int j=0; j < 4; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1. (object colors for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        object_colors[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  object.shapes.resize(1);
  object.poses.resize(1);
  object.shapes[0].dimensions.resize(3);

  bool apply_offset_to_collision_objects;
  std::vector<double> collision_object_offset;
  geometry_msgs::Pose collision_object_offset_pose;

  ros::NodeHandle ph("~");
  ph.param("apply_offset_to_collision_objects",apply_offset_to_collision_objects,false);

  if(apply_offset_to_collision_objects)
  {
    ROS_WARN("Applying offset to the collision objects...");

    if(ph.hasParam("collision_object_offset"))
    {
      XmlRpc::XmlRpcValue plist;
      std::string p;
      collision_object_offset.clear();
      ph.getParam("collision_object_offset/xyz", plist);
      std::stringstream ss(plist);
      while(ss >> p)
        collision_object_offset.push_back(atof(p.c_str()));

      ph.getParam("collision_object_offset/rpy", plist);
      std::stringstream ss1(plist);
      while(ss1 >> p)
        collision_object_offset.push_back(atof(p.c_str()));

      collision_object_offset_pose.position.x = collision_object_offset[0];
      collision_object_offset_pose.position.y = collision_object_offset[1];
      collision_object_offset_pose.position.z = collision_object_offset[2];
      leatherman::rpyToQuatMsg(collision_object_offset[3], collision_object_offset[4], collision_object_offset[5], collision_object_offset_pose.orientation);
    }
  }

  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    object.header.frame_id = "base_footprint";
    object.header.stamp = ros::Time::now();

    object.poses[0].position.x = objects[i][0];
    object.poses[0].position.y = objects[i][1];
    object.poses[0].position.z = objects[i][2];
    object.poses[0].orientation.x = 0;
    object.poses[0].orientation.y = 0;
    object.poses[0].orientation.z = 0;
    object.poses[0].orientation.w = 1;

    object.shapes[0].dimensions[0] = objects[i][3];
    object.shapes[0].dimensions[1] = objects[i][4];
    object.shapes[0].dimensions[2] = objects[i][5];


    // apply collision object offset
    // right now just translates and rotates about z
    if(apply_offset_to_collision_objects)
    {
      if(!collision_object_offset.empty())
      {
        geometry_msgs::Pose p, p2;
        Eigen::Affine3d a;
        a(0,0) = cos(collision_object_offset[5]);
        a(1,0) = sin(collision_object_offset[5]);
        a(0,1) = -sin(collision_object_offset[5]);
        a(1,1) = cos(collision_object_offset[5]);
        leatherman::msgFromPose(a, p2);
        leatherman::multiplyPoses(p2, object.poses[0], p);
        p.position.x += collision_object_offset_pose.position.x;
        p.position.y += collision_object_offset_pose.position.y;
        p.position.z += collision_object_offset_pose.position.z;
        object.poses[0] = p;
      }
      else
        ROS_ERROR("Expecting to translate/rotate collision objects in robot frame but offset not found.");
    }

    collision_objects.push_back(object);
    ROS_DEBUG("[%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);

    std::vector<double> dim(3,0);
    dim[0] = objects[i][3];
    dim[1] = objects[i][4];
    dim[2] = objects[i][5];
    marker = viz::getCubeMarker(object.poses[0], dim, object_colors[i], "base_footprint", "collision_objects", int(i));
    ma.markers.push_back(marker);
  }
  return true;
}

bool getKnownObjectsFromFile(std::string objects_filename, std::vector<arm_navigation_msgs::CollisionObject> &objects, visualization_msgs::MarkerArray &ma)
{
  if(!objects_filename.empty())
  {
    if(!getCollisionObjects(objects_filename, objects, ma))
    {
      ROS_ERROR("Failed to get the known collision objects. (%s)", objects_filename.c_str());
      return false;
    }
  }
  return true;
}

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
  PViz pviz;
  ros::NodeHandle nh, ph("~");
  sleep(1);
  bool restrict_shoulder_pan, compare_collision_models, test_timing, visualize, multi_level_check, low_res;
  std::string group_name, world_frame, robot_description, known_objects_filename;
  std::vector<double> dims(3, 0.0), origin(3, 0.0);
  std::vector<std::string> r_joint_names, l_joint_names, r_finger_names, l_finger_names;
  std::vector<double> r_min_limits, r_max_limits, l_min_limits, l_max_limits;
  std::vector<bool> r_continuous, l_continuous;
  std_msgs::ColorRGBA white; white.r = 1; white.g = 1; white.b = 1; white.a = 0.4;
  std::vector<double> black(4,0); black[3] = 1.0;
  visualization_msgs::MarkerArray ma;
  ph.param("restrict_shoulder_pan", restrict_shoulder_pan, true);
  ph.param<std::string>("group_name", group_name, "");
  ph.param<std::string>("world_frame", world_frame, "");
  ph.param<std::string>("known_objects_filename", known_objects_filename, "");
  ph.param("perform_a_timing_test", test_timing, true);
  ph.param("perform_a_collision_model_fidelity_comparison", compare_collision_models, false);
  ph.param("visualize", visualize, false);
  ph.param("low_res", low_res, true);
  ph.param("multi_level_check", multi_level_check, false);
  pviz.setReferenceFrame(world_frame);
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

  // cause more collisions
  if(restrict_shoulder_pan)
  {
    ROS_WARN("Restricting the shoulder motion.");
    r_min_limits[0] = 0;
    l_max_limits[0] = 0;
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

  visualization_msgs::MarkerArray cma;
  if(!getKnownObjectsFromFile(known_objects_filename, scene->collision_objects, cma))
  {
    ROS_ERROR("Failed to retrieve collision objects from file.");
    return 0;
  }
  cspace->setPlanningScene(*scene);

  ros::spinOnce();
  p.publish(cspace->getVisualization("distance_field"));
  p.publish(cspace->getVisualization("bounds"));
  p.publish(cspace->getVisualization("collision_objects"));
  p.publish(cma);

  // prepare planning scene for non-planning arm
  arm_navigation_msgs::RobotState robot_state;
  for(size_t j = 0; j < l_joint_names.size(); ++j)
    robot_state.joint_state.name.push_back(l_joint_names[j]);
  robot_state.joint_state.position.resize(l_joint_names.size());

  BodyPose bp(0,0,0.29,0);
  geometry_msgs::Pose text_pose; text_pose.orientation.w = 1.0;
  text_pose.position.x = 0.2; text_pose.position.y = -0.6; text_pose.position.z = 1.4;
  std::vector<double> langles, rangles, trial_time, ccps;

  ROS_INFO("-------------------------------------------------------------------------");
  ROS_INFO("restrict_shoulder_pan: %d", restrict_shoulder_pan);
  ROS_INFO("timing_test: %d", test_timing);
//  ROS_INFO("   multi_level_check: %d", multi_level_check);
//  ROS_INFO("             low_res: %d (matters if multi_level_check is disabled)", low_res);
  ROS_INFO("collision_model_test:  %d", compare_collision_models);
  ROS_INFO("visualize:  %d", visualize);
  ROS_INFO("-------------------------------------------------------------------------");
  double prep_time, ptps;
  std::vector<std::vector<std::vector<KDL::Frame> > > frames;

  /*
  if(test_timing)
  {
    clock_t start = clock();
      langles = getRandomJointPositions(l_min_limits, l_max_limits, l_continuous);
      rangles = getRandomJointPositions(r_min_limits, r_max_limits, r_continuous);

      for(size_t j = 0; j < langles.size(); ++j)
        robot_state.joint_state.position[j] = langles[j];
    for(size_t i = 0; i < NUM_TIMING_SAMPLES; ++i)
    {
      cspace->setRobotState(robot_state);
    }
    prep_time = (clock()-start)/(double)CLOCKS_PER_SEC;
    ptps = (double(NUM_TIMING_SAMPLES)/prep_time);
    ROS_INFO("Prepared %d samples for collision checking in %0.5f seconds.  (checks_per_sec: %0.1f)", NUM_TIMING_SAMPLES, prep_time, ptps);
  }
  */

  std::string test_name = " ";
  std::vector<std::vector<double> > langlesv(NUM_TIMING_SAMPLES), ranglesv(NUM_TIMING_SAMPLES);
  for(size_t k = 0; k < NUM_TRIALS; ++k)
  {
    // Generate all random samples for this trial
    clock_t start = clock();
    for(size_t i = 0; i < NUM_TIMING_SAMPLES; ++i)
    {
      langlesv[i] = getRandomJointPositions(l_min_limits, l_max_limits, l_continuous);
      ranglesv[i] = getRandomJointPositions(r_min_limits, r_max_limits, r_continuous);
    }
    prep_time = (clock()-start)/(double)CLOCKS_PER_SEC;
    ptps = (double(NUM_TIMING_SAMPLES)/prep_time);
    ROS_INFO("Computed %d random samples for collision checking in %0.5f seconds.  (samples_per_sec: %0.1f)", NUM_TIMING_SAMPLES, prep_time, ptps);

    // for Timing Test, run 3 times with different params
    for(size_t w = 0; w < 3; ++w)
    {
      // for collision model test, just run it once
      if(!test_timing && w > 1)
        break;

      if(w == 0)    // multi_res
      {
        test_name = "multi_level";
        multi_level_check = true;
      }
      else if(w == 1) // low_res
      {
        test_name = "low_res";
        multi_level_check = false;
        low_res = true;
      }
      else if(w == 2) // high_res
      {
        test_name = "high_res";
        multi_level_check = false;
        low_res = false;
      }

      int lrv_not_hrv = 0, hrv_not_lrv = 0, hrv_equals_lrv = 0;
      int invalid = 0, valid = 0, lr_invalid = 0, lr_valid = 0;
      start = clock();
      for(size_t i = 0; i < NUM_TIMING_SAMPLES; ++i)
      {
        for(size_t j = 0; j < langlesv[i].size(); ++j)
          robot_state.joint_state.position[j] = langlesv[i][j];
        cspace->setRobotState(robot_state);

        if(DEBUG)
        {
          ROS_INFO("[ left] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", langlesv[i][0], langlesv[i][1], langlesv[i][2], langlesv[i][3], langlesv[i][4], langlesv[i][5], langlesv[i][6]);
          ROS_INFO("[right] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", ranglesv[i][0], ranglesv[i][1], ranglesv[i][2], ranglesv[i][3], ranglesv[i][4], ranglesv[i][5], ranglesv[i][6]);
        }


        // Timing test: Compute # collisions per second
        if(test_timing) 
        {
          if(!frames.empty())
            frames[0].clear();

          if(multi_level_check)
          {
            if(!cspace->isStateValid(ranglesv[i], frames, false, visualize, dist))
              invalid++;
            else
              valid++;
          }
          else
          {
            if(!cspace->checkCollision(ranglesv[i], frames, low_res, false, visualize, dist))
              invalid++;
            else
              valid++;
          }
        }

        // Collision Model Fidelity Test: Compare the two collision models
        if(compare_collision_models)
        {
          bool lrv = true; bool hrv = true;
          if(!cspace->checkCollision(ranglesv[i], true, false, visualize, dist))
          {
            lrv = false;
            lr_invalid++;
          }
          else
            lr_valid++;

          if(!cspace->checkCollision(ranglesv[i], false, false, visualize, dist))
          {
            hrv = false;
            invalid++;
          }
          else
            valid++;
          // check if the collision check results make sense
          if(lrv && (!hrv))
          {
            lrv_not_hrv++;
            ROS_ERROR("   [%d] low-res model is VALID while the high-res model is INVALID.", int(i));

            if(visualize)
            {
              ROS_INFO("low_res:");
              cspace->checkCollision(ranglesv[i], true, true, true, dist);

              ROS_INFO("high_res: ");
              cspace->checkCollision(ranglesv[i], false, true, true, dist);

              pviz.deleteVisualizations("collision_spheres", 100);
              pviz.visualizeRobot(ranglesv[i], langlesv[i], bp, 0, "robot", 0, true);
              p.publish(cspace->getVisualization("collision_model"));
              ma = cspace->getVisualization("low_res_collision_model");
              for(size_t y = 0; y <  ma.markers.size(); ++y)
                ma.markers[y].color = white;
              p.publish(ma);
              p.publish(cspace->getVisualization("collisions"));
              pviz.publish(viz::getTextMarker(text_pose, boost::lexical_cast<std::string>(i), 0.2, black, "base_footprint", "sample", 0));
              ros::Duration(1.5).sleep();
            }
          }
          else if((!lrv) && hrv)
          {
            hrv_not_lrv++;
            ROS_DEBUG("   [%d] low-res model is INVALID while the high-res model is VALID.", int(i));
          }
          else if(lrv == hrv)
          {
            hrv_equals_lrv++;
          }
        }

        // visualize
        if(visualize)
        {
          pviz.deleteVisualizations("collision_spheres", 100);
          pviz.deleteVisualizations("collision_model", 100);
          pviz.deleteVisualizations("low_res_collision_model", 100);
          usleep(5000);

          pviz.visualizeRobot(ranglesv[i], langlesv[i], bp, 0, "robot", 0, true);
          if(low_res)
            p.publish(cspace->getVisualization("low_res_collision_model"));
          else
            p.publish(cspace->getVisualization("collision_model"));

          p.publish(cspace->getVisualization("collisions"));
          ros::Duration(1.5).sleep();
        }
      }

      if(test_timing)
      {
        trial_time.push_back((clock()-start)/(double)CLOCKS_PER_SEC);
        ccps.push_back(double(NUM_TIMING_SAMPLES)/trial_time.back());
        ROS_INFO("[trial %d] [%11s] Collision checked %d samples in %0.5f seconds.  (valid: %d  invalid: %d  checks_per_sec: %0.1f)", int(k), test_name.c_str(), NUM_TIMING_SAMPLES, trial_time.back(), valid, invalid, ccps.back());
      }

      if(compare_collision_models)
        ROS_INFO("[%d] [low_res] valid: %6d  invalid: %6d  [high_res] valid: %6d  invalid: %6d (diff: %d  high_valid-low_invalid: %d  low_valid-high_invalid: %d  high_valid==low_valid: %d)", int(k), lr_valid, lr_invalid, valid, invalid, abs(valid-lr_valid), hrv_not_lrv, lrv_not_hrv, hrv_equals_lrv);
    }
  }

  ros::spinOnce();
  sleep(1);

  ROS_INFO("Done");
  delete cspace;
  delete grid;
  delete df;
  return 0;
}

