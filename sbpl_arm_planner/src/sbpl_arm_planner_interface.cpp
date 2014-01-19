/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <visualization_msgs/Marker.h>

clock_t starttime;

using namespace sbpl_arm_planner;

SBPLArmPlannerInterface::SBPLArmPlannerInterface(RobotModel *rm, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df) : 
  nh_("~"), planner_(NULL), sbpl_arm_env_(NULL), prm_(NULL)
{
  rm_ = rm;
  cc_ = cc;
  as_ = as;
  df_ = df;
  planner_initialized_ = false;
}

SBPLArmPlannerInterface::~SBPLArmPlannerInterface()
{
  if(planner_ != NULL)
    delete planner_;
  if(sbpl_arm_env_ != NULL)
    delete sbpl_arm_env_;
  if(prm_ != NULL)
    delete prm_;
}

bool SBPLArmPlannerInterface::init(std::string ns)
{
  if(!initializePlannerAndEnvironment(ns))
    return false;

  planner_initialized_ = true;
  ROS_INFO("The SBPL arm planner node initialized succesfully.");
  return true;
}

bool SBPLArmPlannerInterface::initializePlannerAndEnvironment(std::string ns)
{
  prm_ = new sbpl_arm_planner::PlanningParams();
  if(!prm_->init(ns))
    return false;

  grid_ = new sbpl_arm_planner::OccupancyGrid(df_);
  sbpl_arm_env_ = new sbpl_arm_planner::EnvironmentROBARM3D(grid_, rm_, cc_, as_, prm_);

  if(!sbpl_arm_env_)
    return false;

  if(!as_->init(sbpl_arm_env_))
  {
    ROS_ERROR("Failed to initialize the action set.");
    return false;
  } 
  //as_->print();

  //initialize environment  
  planner_ = new ARAPlanner(sbpl_arm_env_, true);

  //initialize arm planner environment
  if(!sbpl_arm_env_->initEnvironment())
  {
    ROS_ERROR("ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP 
  if(!sbpl_arm_env_->InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
    return false;
  }

  //set epsilon
  //TODO: get from params
  planner_->set_initialsolution_eps(100.0);

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(prm_->search_mode_);

  ROS_INFO("Initialized sbpl arm planning environment.");
  return true;
}

bool SBPLArmPlannerInterface::solve(const arm_navigation_msgs::GetMotionPlan::Request &req,
                                    arm_navigation_msgs::GetMotionPlan::Response &res) 
{
  // this version of solve doesn't recompute the distance field

  if(!planner_initialized_)
    return false;

  if(req.motion_plan_request.goal_constraints.position_constraints.empty())
    return false;

  // preprocess
  clock_t t_preprocess = clock();
  prm_->planning_frame_ = req.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id;
  grid_->setReferenceFrame(prm_->planning_frame_); 
  cc_->setRobotState(req.motion_plan_request.start_state);
  // TODO: set kinematics to planning frame
  double preprocess_time = (clock() - t_preprocess) / (double)CLOCKS_PER_SEC;

  // plan
  clock_t t_plan = clock();
  res.robot_state = req.motion_plan_request.start_state;
  if(!planToPosition(req,res))
    return false;

  res_ = res;
  double plan_time = (clock() - t_plan) / (double)CLOCKS_PER_SEC;
  ROS_INFO("t_plan: %0.3fsec  t_preprocess: %0.3fsec", plan_time, preprocess_time);
  return true;
}

bool SBPLArmPlannerInterface::solve(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene,
                                    const arm_navigation_msgs::GetMotionPlan::Request &req,
                                    arm_navigation_msgs::GetMotionPlan::Response &res) 
{
  if(!planner_initialized_)
    return false;

  // preprocess
  clock_t t_preprocess = clock();
  cc_->setPlanningScene(*planning_scene); 
  prm_->planning_frame_ = planning_scene->collision_map.header.frame_id;
  grid_->setReferenceFrame(prm_->planning_frame_);
  // TODO: set kinematics to planning frame
  double preprocess_time = (clock() - t_preprocess) / (double)CLOCKS_PER_SEC;

  // plan
  clock_t t_plan = clock();
  res.robot_state = planning_scene->robot_state;
  if(!planToPosition(req,res))
    return false;

  res_ = res;
  double plan_time = (clock() - t_plan) / (double)CLOCKS_PER_SEC;
  ROS_INFO("t_plan: %0.3fsec  t_preprocess: %0.3fsec", plan_time, preprocess_time);
  return true;
}

bool SBPLArmPlannerInterface::setStart(const sensor_msgs::JointState &state)
{
  std::vector<double> initial_positions;
  if(!leatherman::getJointPositions(state, prm_->planning_joints_, initial_positions))
  {
    ROS_ERROR("Start state does not contain the positions of the planning joints.");
    return false;
  }

  if(sbpl_arm_env_->setStartConfiguration(initial_positions) == 0)
  {
    ROS_ERROR("Environment failed to set start state. Not Planning.");
    return false;
  }
  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state. Not Planning.");
    return false;
  }
  ROS_DEBUG("start: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", initial_positions[0],initial_positions[1],initial_positions[2],initial_positions[3],initial_positions[4],initial_positions[5],initial_positions[6]);
  return true;
}

bool SBPLArmPlannerInterface::setGoalPosition(const arm_navigation_msgs::Constraints &goals)
{
  geometry_msgs::Quaternion goalq;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (12,0));  //Changed to include Quaternion, and FA
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  if(goals.position_constraints.size() != goals.orientation_constraints.size())
    ROS_WARN("There are %d position contraints and %d orientation constraints.", int(goals.position_constraints.size()),int(goals.orientation_constraints.size()));

  //currently only supports one goal
  sbpl_goal[0][0] = goals.position_constraints[0].position.x;
  sbpl_goal[0][1] = goals.position_constraints[0].position.y;
  sbpl_goal[0][2] = goals.position_constraints[0].position.z;

  //convert quaternion into roll,pitch,yaw //TODO
  goalq = goals.orientation_constraints[0].orientation;
  goalq.w += 0.001; //perturb quaternion if rpy will suffer from gimbal lock
  leatherman::getRPY(goalq, sbpl_goal[0][3], sbpl_goal[0][4], sbpl_goal[0][5]);
 
  // set sbpl_arm_planner::GoalType
  if(!goals.joint_constraints.empty())
  {
    sbpl_goal[0][6] = GoalType::XYZ_RPY_FA_GOAL;
    sbpl_goal[0][11] = goals.joint_constraints[0].position;
  }
  else
    sbpl_goal[0][6] = GoalType::XYZ_RPY_GOAL;


  //orientation constraint as a quaternion 
  sbpl_goal[0][7] = goals.orientation_constraints[0].orientation.x;
  sbpl_goal[0][8] = goals.orientation_constraints[0].orientation.y;
  sbpl_goal[0][9] = goals.orientation_constraints[0].orientation.z;
  sbpl_goal[0][10] = goals.orientation_constraints[0].orientation.w;

  //allowable tolerance from goal
  if(goals.position_constraints[0].constraint_region_shape.dimensions.size() == 3)
  {
    sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[1];
    sbpl_tolerance[0][2] = goals.position_constraints[0].constraint_region_shape.dimensions[2];
  }
  else
  {
    sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
  }
  sbpl_tolerance[0][3] = goals.orientation_constraints[0].absolute_roll_tolerance;
  sbpl_tolerance[0][4] = goals.orientation_constraints[0].absolute_pitch_tolerance;
  sbpl_tolerance[0][5] = goals.orientation_constraints[0].absolute_yaw_tolerance;

  ROS_INFO("goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)  (quat: %0.3f %0.3f %0.3f %0.3f)", prm_->planning_frame_.c_str(),sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1], goals.orientation_constraints[0].orientation.x, goals.orientation_constraints[0].orientation.y, goals.orientation_constraints[0].orientation.z, goals.orientation_constraints[0].orientation.w);

  //set sbpl environment goal
  if(!sbpl_arm_env_->setGoalPosition(sbpl_goal, sbpl_tolerance))
  {
    ROS_ERROR("Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }

  //set planner goal	
  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state. Exiting.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerInterface::planKinematicPath(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
  if(!planner_initialized_)
  {
    ROS_ERROR("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
    return false;
  }

  if(req.motion_plan_request.goal_constraints.position_constraints.empty())
  {
    ROS_ERROR("There aren't any goal pose constraints in the request message. We need those to plan :).");
    return false;
  }

  if(!planToPosition(req, res))
    return false;

  return true;
}

bool SBPLArmPlannerInterface::plan(trajectory_msgs::JointTrajectory &traj)
{
  bool b_ret = false;
  std::vector<int> solution_state_ids;

  //reinitialize the search space
  planner_->force_planning_from_scratch();

  //plan
  
  ReplanParams replan_params(prm_->allowed_time_);
  replan_params.initial_eps = 100.0;
  replan_params.final_eps = 100.0;
  replan_params.dec_eps = 10.0;
  replan_params.return_first_solution = false;

  //b_ret = planner_->replan(prm_->allowed_time_, &solution_state_ids, &solution_cost_);
  b_ret = planner_->replan(&solution_state_ids, replan_params, &solution_cost_);

  //check if an empty plan was received.
  if(b_ret && solution_state_ids.size() <= 0)
  {
    ROS_WARN("Path returned by the planner is empty?");
    b_ret = false;
  }

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids.size() > 0))
  {
    ROS_INFO("Initial Epsilon: %0.3f   Final Epsilon: %0.3f  Solution Cost: %d", planner_->get_initial_eps(),planner_->get_final_epsilon(), solution_cost_);
    
    if(!sbpl_arm_env_->convertStateIDPathToJointTrajectory(solution_state_ids, traj))
      return false;
  }
  return b_ret;
}

bool SBPLArmPlannerInterface::planToPosition(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
  starttime = clock();
  int status = 0;
  prm_->allowed_time_ = req.motion_plan_request.allowed_planning_time.toSec();
  req_ = req.motion_plan_request;

  if(!canServiceRequest(req))
    return false;

  //TODO: transform goal pose into reference_frame
  arm_navigation_msgs::Constraints goal_constraints = req.motion_plan_request.goal_constraints;
  /*
  geometry_msgs::Pose gpose, gpose_out;
  gpose.position = req.motion_plan_request.goal_constraints.position_constraints[0].position;
  gpose.orientation = req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;
  sbpl_arm_planner::transformPose(pscene_, gpose, gpose_out, req.motion_plan_request.goal_constraints[0].position_constraints[0].header.frame_id, prm_->planning_frame);
  goal_constraints.orientation_constraints[0].orientation = gpose_out.orientation;
  */

  // set start
  ROS_DEBUG("Setting start.");
  if(!setStart(req.motion_plan_request.start_state.joint_state))
  {
    status = -1;
    ROS_ERROR("Failed to set initial configuration of robot.");
  }

  // set goal
  ROS_DEBUG("Setting goal.");
  if(!setGoalPosition(goal_constraints) && status == 0)
  {
    status = -2;
    ROS_ERROR("Failed to set goal position.");
  }
  
  // plan 
  ROS_DEBUG("Calling planner"); 
  if(status == 0 && plan(res.trajectory.joint_trajectory))
  {
    res.trajectory.joint_trajectory.header.seq = req.motion_plan_request.goal_constraints.position_constraints[0].header.seq; 
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    // fill in the waypoint times (not very intelligently)
    res.trajectory.joint_trajectory.points[0].time_from_start.fromSec(prm_->waypoint_time_);
    for(size_t i = 1; i < res.trajectory.joint_trajectory.points.size(); i++)
      res.trajectory.joint_trajectory.points[i].time_from_start.fromSec(res.trajectory.joint_trajectory.points[i-1].time_from_start.toSec() + prm_->waypoint_time_);

    res.planning_time = ros::Duration((clock() - starttime) / (double)CLOCKS_PER_SEC);

    // shortcut path
    if(prm_->shortcut_path_)
    {
      trajectory_msgs::JointTrajectory straj;
      if(!interpolateTrajectory(cc_, res.trajectory.joint_trajectory.points, straj.points))
        ROS_WARN("Failed to interpolate planned trajectory with %d waypoints before shortcutting.", int(res.trajectory.joint_trajectory.points.size()));
      
      shortcutTrajectory(cc_, straj.points,res.trajectory.joint_trajectory.points);
    }

    // interpolate path
    if(prm_->interpolate_path_)
    {
      trajectory_msgs::JointTrajectory itraj = res.trajectory.joint_trajectory;
      interpolateTrajectory(cc_, itraj.points, res.trajectory.joint_trajectory.points);
    }

    if(prm_->print_path_)
      leatherman::printJointTrajectory(res.trajectory.joint_trajectory, "path");
  }
  else
  {
    status = -3;
    ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds).", prm_->allowed_time_);
  }

  if(status == 0)
    return true;

  return false;
}

bool SBPLArmPlannerInterface::canServiceRequest(const arm_navigation_msgs::GetMotionPlan::Request &req)
{
  // check for an empty start state
  if(req.motion_plan_request.start_state.joint_state.position.empty())
  {
    ROS_ERROR("No start state given. Unable to plan.");
    return false;
  }

  // check if position & orientation constraints is empty
  if(req.motion_plan_request.goal_constraints.position_constraints.empty() || 
      req.motion_plan_request.goal_constraints.orientation_constraints.empty())
  {
    ROS_ERROR("Position or orientation constraint is empty. Expecting a 6D end effector pose constraint. Exiting.");
    return false;
  }
  
  // check if there is more than one goal constraint
  if(req.motion_plan_request.goal_constraints.position_constraints.size() > 1 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() > 1)
    ROS_WARN("The planning request message contains %d position and %d orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", int(req.motion_plan_request.goal_constraints.position_constraints.size()), int(req.motion_plan_request.goal_constraints.orientation_constraints.size()));

  return true;
}

std::map<std::string, double>  SBPLArmPlannerInterface::getPlannerStats()
{
  std::map<std::string, double> stats;
  stats["initial solution planning time"] = planner_->get_initial_eps_planning_time();
  stats["initial epsilon"] = planner_->get_initial_eps();
  stats["initial solution expansions"] = planner_->get_n_expands_init_solution();
  stats["final epsilon planning time"] = planner_->get_final_eps_planning_time();
  stats["final epsilon"] = planner_->get_final_epsilon();
  stats["solution epsilon"] = planner_->get_solution_eps();
  stats["expansions"] = planner_->get_n_expands();
  stats["solution cost"] = solution_cost_;
  return stats;
}

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getCollisionModelTrajectoryMarker()
{
  visualization_msgs::MarkerArray ma, ma1;
  std::vector<std::vector<double> > traj;

  if(res_.trajectory.joint_trajectory.points.empty())
  {
    ROS_ERROR("No trajectory found to visualize yet. Plan a path first.");
    return ma;
  }

  traj.resize(res_.trajectory.joint_trajectory.points.size());
  double cinc = 1.0/double(res_.trajectory.joint_trajectory.points.size());
  for(size_t i = 0; i < res_.trajectory.joint_trajectory.points.size(); ++i)
  {
    traj[i].resize(res_.trajectory.joint_trajectory.points[i].positions.size());
    for(size_t j = 0; j < res_.trajectory.joint_trajectory.points[i].positions.size(); j++)
      traj[i][j] = res_.trajectory.joint_trajectory.points[i].positions[j];

    ma1 = cc_->getCollisionModelVisualization(traj[i]);

    for(size_t j = 0; j < ma1.markers.size(); ++j)
    {
      ma1.markers[j].color.r = 0.1;
      ma1.markers[j].color.g = cinc*double(res_.trajectory.joint_trajectory.points.size()-(i+1));
      ma1.markers[j].color.b = cinc*double(i);
    }
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
  }

  for(size_t i = 0; i < ma.markers.size(); ++i)
  {
    ma.markers[i].ns = "trajectory";
    ma.markers[i].id = i;
  }

  return ma;
}

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getVisualization(std::string type)
{
  visualization_msgs::MarkerArray ma;
  if(type.compare("goal") == 0)
  {
    if(req_.goal_constraints.position_constraints.empty())
    {
      ROS_WARN("Failed to get visualization marker for goals because no position constraints found.");
      return visualization_msgs::MarkerArray();
    }

    std::vector<std::vector<double> > poses(req_.goal_constraints.position_constraints.size(),std::vector<double>(6,0));
    for(size_t i = 0; i < req_.goal_constraints.position_constraints.size(); ++i)
    {
      poses[i][0] = req_.goal_constraints.position_constraints[i].position.x;
      poses[i][1] = req_.goal_constraints.position_constraints[i].position.y;
      poses[i][2] = req_.goal_constraints.position_constraints[i].position.z;

      if(req_.goal_constraints.orientation_constraints.size() > i)
        leatherman::getRPY(req_.goal_constraints.orientation_constraints[i].orientation, poses[i][3], poses[i][4], poses[i][5]);
      else
      {
        poses[i][3] = 0;
        poses[i][4] = 0;
        poses[i][5] = 0;
      }
    }
    ma = viz::getPosesMarkerArray(poses, req_.goal_constraints.position_constraints[0].header.frame_id, "goals", 0);
  }
  else if(type.compare("expansions") == 0)
  {
    std::vector<std::vector<double> > expanded_states;
    sbpl_arm_env_->getExpandedStates(&(expanded_states));
    if(!expanded_states.empty())
    {
      std::vector<std::vector<double> > colors(2, std::vector<double>(4,0));
      colors[0][0] = 1;
      colors[0][3] = 1;
      colors[1][1] = 1;
      colors[1][3] = 1;
      ma = viz::getCubesMarkerArray(expanded_states, 0.01, colors, prm_->planning_frame_, "expansions", 0);
    }
  }
  else
    ma = sbpl_arm_env_->getVisualization(type);

  return ma;
}

