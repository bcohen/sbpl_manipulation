/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *********************************************************************/

#include <ros/ros.h>
#include <signal.h>
#include <stdlib.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <sbpl_manipulation_components_pr2/ubr1_kdl_robot_model.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>

using namespace sbpl_arm_planner;

void fillConstraint(const std::vector<double> &pose, std::string frame_id, arm_navigation_msgs::Constraints &goals)
{

  if(pose.size() < 6)
    return;

  goals.position_constraints.resize(1);
  goals.orientation_constraints.resize(1);
  goals.position_constraints[0].header.frame_id = frame_id;
  goals.position_constraints[0].position.x = pose[0];
  goals.position_constraints[0].position.y = pose[1];
  goals.position_constraints[0].position.z = pose[2];
  leatherman::rpyToQuatMsg(pose[3], pose[4], pose[5], goals.orientation_constraints[0].orientation);

  geometry_msgs::Pose p;
  p.position = goals.position_constraints[0].position;
  p.orientation = goals.orientation_constraints[0].orientation;
  leatherman::printPoseMsg(p, "Goal");

  goals.position_constraints[0].constraint_region_shape.dimensions.resize(3, 0.015);
  goals.orientation_constraints[0].absolute_roll_tolerance = 0.05;
  goals.orientation_constraints[0].absolute_pitch_tolerance = 0.05;
  goals.orientation_constraints[0].absolute_yaw_tolerance = 0.05;
  ROS_INFO("Done packing the goal constraints message.");
}

arm_navigation_msgs::CollisionObject getCollisionCube(geometry_msgs::Pose pose, std::vector<double> &dims, std::string frame_id, std::string id)
{
  arm_navigation_msgs::CollisionObject object;
  object.id = id;
  object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  object.header.frame_id = frame_id;
  object.header.stamp = ros::Time::now();

  arm_navigation_msgs::Shape box_object;
  box_object.type = arm_navigation_msgs::Shape::BOX;
  box_object.dimensions.resize(3);
  box_object.dimensions[0] = dims[0];
  box_object.dimensions[1] = dims[1];
  box_object.dimensions[2] = dims[2];

  object.shapes.push_back(box_object);
  object.poses.push_back(pose);
  return object;
}

std::vector<arm_navigation_msgs::CollisionObject> getCollisionCubes(std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids, std::string frame_id)
{
  std::vector<arm_navigation_msgs::CollisionObject> objs;
  std::vector<double> dims(3,0);
  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return objs;
  }

  for(size_t i = 0; i < objects.size(); i++)
  {
    pose.position.x = objects[i][0];
    pose.position.y = objects[i][1];
    pose.position.z = objects[i][2];
    dims[0] = objects[i][3];
    dims[1] = objects[i][4];
    dims[2] = objects[i][5];

    objs.push_back(getCollisionCube(pose, dims, frame_id, object_ids.at(i)));
  }
  return objs;
}

std::vector<arm_navigation_msgs::CollisionObject> getCollisionObjects(std::string filename, std::string frame_id)
{
  char sTemp[1024];
  int num_obs = 0;
  std::vector<std::string> object_ids;
  std::vector<std::vector<double> > objects;
  std::vector<arm_navigation_msgs::CollisionObject> objs;

  char* file = new char[filename.length()+1];
  filename.copy(file, filename.length(),0);
  file[filename.length()] = '\0';
  FILE* fCfg = fopen(file, "r");

  if(fCfg == NULL)
  {
    ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
    return objs;
  }

  // get number of objects
  if(fscanf(fCfg,"%s",sTemp) < 1)
    printf("Parsed string has length < 1.\n");

  num_obs = atoi(sTemp);

  ROS_INFO("%i objects in file",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs);
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      printf("Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    objects[i].resize(6);
    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1.\n");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
  }

  return getCollisionCubes(objects, object_ids, frame_id);
}

bool getInitialConfiguration(ros::NodeHandle &nh, arm_navigation_msgs::RobotState &state)
{
  XmlRpc::XmlRpcValue xlist;

  //joint_state
  if(nh.hasParam("initial_configuration/joint_state"))
  {
    nh.getParam("initial_configuration/joint_state", xlist);

    if(xlist.getType() != XmlRpc::XmlRpcValue::TypeArray)
      ROS_WARN("initial_configuration/joint_state is not an array.");

    if(xlist.size() == 0)
      return false;
    std::cout<<xlist <<endl;
    for(int i = 0; i < xlist.size(); ++i)
    {
      state.joint_state.name.push_back(std::string(xlist[i]["name"]));

      if(xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        state.joint_state.position.push_back(double(xlist[i]["position"]));
      else
      {
        ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
        if(xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          int pos = xlist[i]["position"];
          state.joint_state.position.push_back(double(pos));
        }
      }
    }
  }
  else
  {
    ROS_ERROR("initial_configuration/joint_state is not on the param server.");
    return false;
  }
  
  //multi_dof_joint_state
  if(nh.hasParam("initial_configuration/multi_dof_joint_state"))
  {
    nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

    if(xlist.getType() != XmlRpc::XmlRpcValue::TypeArray)
      ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");

    if(xlist.size() != 0)
    {
      geometry_msgs::Pose pose;
      for(int i = 0; i < xlist.size(); ++i)
      {
        state.multi_dof_joint_state.frame_ids.push_back(xlist[i]["frame_id"]);
        state.multi_dof_joint_state.child_frame_ids.push_back(xlist[i]["child_frame_id"]);
        pose.position.x = xlist[i]["x"];
        pose.position.y = xlist[i]["y"];
        pose.position.z = xlist[i]["z"];
        leatherman::rpyToQuatMsg(xlist[i]["roll"], xlist[i]["pitch"], xlist[i]["yaw"], pose.orientation);
        state.multi_dof_joint_state.poses.push_back(pose);
      }
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "sbpl_arm_planner_test");
  ros::NodeHandle nh, ph("~");
  sleep(1);
  ros::spinOnce();
  ros::Publisher ma_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
  std::string group_name, kinematics_frame, planning_frame, planning_link, chain_tip_link;
  std::string object_filename, action_set_filename;
  std::vector<double> pose(6,0), goal(6,0);
  ph.param<std::string>("kinematics_frame", kinematics_frame, "");
  ph.param<std::string>("planning_frame", planning_frame, "");
  ph.param<std::string>("planning_link", planning_link, "");
  ph.param<std::string>("chain_tip_link", chain_tip_link, "");
  ph.param<std::string>("group_name", group_name, "");
  ph.param<std::string>("object_filename", object_filename, "");
  ph.param<std::string>("action_set_filename", action_set_filename, "");
  ph.param("goal/x", goal[0], 0.0);
  ph.param("goal/y", goal[1], 0.0);
  ph.param("goal/z", goal[2], 0.0);
  ph.param("goal/roll", goal[3], 0.0);
  ph.param("goal/pitch", goal[4], 0.0);
  ph.param("goal/yaw", goal[5], 0.0);

  // planning joints
  std::vector<std::string> planning_joints;
  XmlRpc::XmlRpcValue xlist;
  ph.getParam("planning/planning_joints", xlist);
  std::string joint_list = std::string(xlist);
  std::stringstream joint_name_stream(joint_list);
  while(joint_name_stream.good() && !joint_name_stream.eof())
  {
    std::string jname;
    joint_name_stream >> jname;
    if(jname.size() == 0)
      continue;
    planning_joints.push_back(jname);
  }
  std::vector<double> start_angles(planning_joints.size(),0);
  if(planning_joints.size() < 7)
    ROS_ERROR("Found %d planning joints on the param server. I usually expect at least 7 joints...", int(planning_joints.size()));

  // robot description
  std::string urdf;
  nh.param<std::string>("robot_description", urdf, " ");

  // distance field
  distance_field::PropagationDistanceField *df = new distance_field::PropagationDistanceField(3.0, 3.0, 3.0, 0.02, -0.75, -1.25, -1.0, 0.2);
  df->reset();

  // robot model
  RobotModel *rm;
  if(group_name.compare("right_arm") == 0)
    rm = new sbpl_arm_planner::PR2KDLRobotModel();
  else if(group_name.compare("arm") == 0)
    rm = new sbpl_arm_planner::UBR1KDLRobotModel();
  else
    rm  = new sbpl_arm_planner::KDLRobotModel(kinematics_frame, chain_tip_link);
  if(!rm->init(urdf, planning_joints))
  {
    ROS_ERROR("Failed to initialize robot model.");
    return false;
  }
  rm->setPlanningLink(planning_link);
  //ROS_WARN("Robot Model Information");
  //rm->printRobotModelInformation();
  //ROS_WARN(" ");

  // configure this so that it works for all robots
  KDL::Frame f;
  if(group_name.compare("right_arm") == 0) //pr2
  {
    f.p.x(-0.05); f.p.y(1.0); f.p.z(0.789675);
    f.M = KDL::Rotation::Quaternion(0,0,0,1);
    rm->setKinematicsToPlanningTransform(f, planning_frame);
  }
  else if(group_name.compare("right_arm") == 0) //ubr1
  {
    f.p.x(-0.05); f.p.y(0.0); f.p.z(0.26);
    f.M = KDL::Rotation::Quaternion(0,0,0,1);
    rm->setKinematicsToPlanningTransform(f, planning_frame);
  }

  // collision checker
  sbpl_arm_planner::OccupancyGrid *grid = new sbpl_arm_planner::OccupancyGrid(df);
  grid->setReferenceFrame(planning_frame);
  sbpl_arm_planner::CollisionChecker *cc;
  cc = new sbpl_arm_planner::SBPLCollisionSpace(grid);

  if(!cc->init(group_name))
    return false;
  if(!cc->setPlanningJoints(planning_joints))
    return false;

  // action set
  sbpl_arm_planner::ActionSet *as = new sbpl_arm_planner::ActionSet(action_set_filename);

  // planner interface
  sbpl_arm_planner::SBPLArmPlannerInterface *planner = new sbpl_arm_planner::SBPLArmPlannerInterface(rm, cc, as, df);

  if(!planner->init())
    return false;
  
  // collision objects
  arm_navigation_msgs::PlanningScenePtr scene(new arm_navigation_msgs::PlanningScene);
  if(!object_filename.empty())
    scene->collision_objects = getCollisionObjects(object_filename, planning_frame);

  // create goal
  arm_navigation_msgs::GetMotionPlan::Request req;
  arm_navigation_msgs::GetMotionPlan::Response res;
  scene->collision_map.header.frame_id = planning_frame;

  // fill goal state
  fillConstraint(goal, planning_frame, req.motion_plan_request.goal_constraints);
  req.motion_plan_request.allowed_planning_time.fromSec(5.0);

  // fill start state 
  if(!getInitialConfiguration(ph, scene->robot_state))
  {
    ROS_ERROR("Failed to get initial configuration.");
    return 0;
  }
  scene->robot_state.joint_state.header.frame_id = planning_frame;  
  req.motion_plan_request.start_state = scene->robot_state;

  // set planning scene
  //cc->setPlanningScene(*scene);
 
  // plan
  ROS_INFO("Calling solve...");
  if(!planner->solve(scene, req, res))
  {
    ROS_ERROR("Failed to plan.");
  }
  else
    ma_pub.publish(planner->getCollisionModelTrajectoryMarker());

  // visualizations
  ros::spinOnce();
  ma_pub.publish(cc->getVisualization("bounds"));
  ma_pub.publish(cc->getVisualization("distance_field"));
  ma_pub.publish(planner->getVisualization("goal"));
  ma_pub.publish(planner->getVisualization("expansions"));
  ma_pub.publish(cc->getVisualization("collision_objects"));
  //ma_pub.publish(grid->getVisualization("occupied_voxels"));
  ma_pub.publish(cc->getCollisionModelVisualization(start_angles));

  ros::spinOnce();

  sleep(1);
  ROS_INFO("Done");
  return 0;
}

