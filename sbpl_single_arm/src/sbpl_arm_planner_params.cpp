/*
 * Copyright (c) 2010, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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

#include <sbpl_arm_planner/sbpl_arm_planner_params.h>

 
namespace sbpl_arm_planner {

SBPLArmPlannerParams::SBPLArmPlannerParams()
{
  epsilon_ = 10;
  use_multires_mprims_ = true;
  use_bfs_heuristic_ = true;
  use_6d_pose_goal_ = true;
  sum_heuristics_ = false;
  use_uniform_cost_ = true;
  use_ik_ = true;
  use_orientation_solver_ = true;

  verbose_ = false;
  verbose_heuristics_ = false;
  verbose_collisions_ = false;
  angle_delta_ = 360;

  num_mprims_ = 0;
  num_long_dist_mprims_ = 0;
  num_short_dist_mprims_ = 0;
  
  short_dist_mprims_thresh_c_ = 20;
  short_dist_mprims_thresh_m_ = 0.2;
  
  cost_multiplier_ = 1000;
  cost_per_cell_ = 0;
  cost_per_meter_ = 0;

  range1_cost_ = 12;
  range2_cost_ = 7;
  range3_cost_ = 2;

  solve_for_ik_thresh_ = 1000;
  solve_for_ik_thresh_m_= 0.20;

  two_calls_to_op_ = false;
  is_goal_function_ = 0;

  expands_log_ = "expands";
  expands2_log_ = "expands2";
  arm_log_ = "arm";
  ik_log_ = "ik";
  cspace_log_ = "cspace";
  solution_log_ = "solution";

  expands_log_level_ = "info";
  expands2_log_level_ = "info";
  ik_log_level_ = "info";
  arm_log_level_ = "info";
  cspace_log_level_ = "info";
  solution_log_level_ = "info";

  /* cartesian planner */
  xyz_resolution_ = 0.02;
  rpy_resolution_ = 0.034906585; // 2 deg
  fa_resolution_ = 0.0523598776; // 3 deg

  cost_per_second_ = cost_multiplier_;
  time_per_cell_ = 0.05;
  joint_vel_.resize(7, 0.200);
  
  //pulled from URDF 
  double desired_speed_pct = 0.8;
  joint_vel_[0] = 0.6*3.48*desired_speed_pct;
  joint_vel_[1] = 0.6*3.47*desired_speed_pct;
  joint_vel_[2] = 0.6*5.45*desired_speed_pct;
  joint_vel_[3] = 0.6*5.50*desired_speed_pct;
  joint_vel_[4] = 0.6*6.00*desired_speed_pct;
  joint_vel_[5] = 0.6*5.13*desired_speed_pct;
  joint_vel_[6] = 0.6*6.00*desired_speed_pct;

  motion_primitive_type_names_.push_back("long_distance");
  motion_primitive_type_names_.push_back("short_distance");
  motion_primitive_type_names_.push_back("adaptive");
  motion_primitive_type_names_.push_back("snap_to_rpy");
  motion_primitive_type_names_.push_back("snap_to_xyzrpy");
  motion_primitive_type_names_.push_back("snap_to_rpy_then_to_xyz");
  motion_primitive_type_names_.push_back("snap_to_xyz_then_to_rpy");
  motion_primitive_type_names_.push_back("snap_to_rpy_at_start");
  motion_primitive_type_names_.push_back("retract_then_snap_to_rpy_then_snap_to_xyz");
  motion_primitive_type_names_.push_back("retract_then_towards_rpy_then_towards_xyz");
  ROS_INFO("Joint Velocities:  %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", joint_vel_[0],joint_vel_[1], joint_vel_[2],joint_vel_[3],joint_vel_[4],joint_vel_[5],joint_vel_[6]);
}

void SBPLArmPlannerParams::initFromParamServer()
{
  ros::NodeHandle nh("~");

  /* planner */
  nh.param("planner/epsilon",epsilon_, 10.0);
  nh.param("planner/use_bfs_heuristic",use_bfs_heuristic_,true);
  nh.param("planner/use_research_heuristic",use_research_heuristic_,false);
  nh.param("planner/use_multiresolution_motion_primitives",use_multires_mprims_,true);
  nh.param("planner/use_uniform_obstacle_cost", use_uniform_cost_,false);
  nh.param("planner/verbose",verbose_,false);
  nh.param("planner/obstacle_distance_cost_far",range3_cost_,12);
  nh.param("planner/obstacle_distance_cost_mid",range2_cost_,7);
  nh.param("planner/obstacle_distance_cost_near",range1_cost_,2);

  /* research params of the planner */
  nh.param("planner/research/solve_with_ik_threshold",solve_for_ik_thresh_m_,0.15);
  nh.param("planner/research/sum_heuristics",sum_heuristics_,false);
  nh.param("planner/research/short_distance_mprims_threshold",short_dist_mprims_thresh_m_, 0.2);

  /* occupancy grid */
  nh.param("collision_space/resolution",resolution_,0.02);
  nh.param<std::string>("collision_space/reference_frame",reference_frame_,"base_link");
  nh.param("collision_space/occupancy_grid/origin_x",originX_,-0.6);
  nh.param("collision_space/occupancy_grid/origin_y",originY_,-1.15);
  nh.param("collision_space/occupancy_grid/origin_z",originZ_,-0.05);  
  nh.param("collision_space/occupancy_grid/size_x",sizeX_,1.6);
  nh.param("collision_space/occupancy_grid/size_y",sizeY_,1.8);
  nh.param("collision_space/occupancy_grid/size_z",sizeZ_,1.4);
  nh.param<std::string>("collision_space/group_name",group_name_,"");

  /* logging */
  nh.param<std::string>("debug/logging/expands", expands_log_level_, "info");
  nh.param<std::string>("debug/logging/expands2", expands2_log_level_, "info");
  nh.param<std::string>("debug/logging/ik", ik_log_level_, "info");
  nh.param<std::string>("debug/logging/arm_model", arm_log_level_, "info");
  nh.param<std::string>("debug/logging/collisions", cspace_log_level_, "info");
  nh.param<std::string>("debug/logging/solution", solution_log_level_, "info");

  XmlRpc::XmlRpcValue joints_param;
  nh.getParam("planning_joints", joints_param);

  if(joints_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_DEBUG("Planning joints list is not an array.");

  std::string joint_list = std::string(joints_param);
  std::stringstream joint_name_stream(joint_list);
  while(joint_name_stream.good() && !joint_name_stream.eof())
  {
    std::string jname;
    joint_name_stream >> jname;
    if(jname.size() == 0)
      continue;
    planning_joints_.push_back(jname);
  }
  if(environment_type_.compare("cartesian") == 0)
  {
    changeLoggerLevel(std::string("ros.sbpl_cartesian_arm_planner") + std::string(".") + expands_log_, expands_log_level_);
    changeLoggerLevel(std::string("ros.sbpl_cartesian_arm_planner") + std::string(".") + expands2_log_, expands2_log_level_);
    changeLoggerLevel(std::string("ros.sbpl_cartesian_arm_planner") + std::string(".") + solution_log_, solution_log_level_);
  }
  else
  {
    changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands_log_, expands_log_level_);
    changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands2_log_, expands2_log_level_);
    changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + solution_log_, solution_log_level_);
  }
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + ik_log_, ik_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + arm_log_, arm_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + cspace_log_, cspace_log_level_);
}

bool SBPLArmPlannerParams::initFromParamFile(std::string param_file)
{
  char* filename = new char[param_file.length()+1];
  param_file.copy(filename, param_file.length(),0);
  filename[param_file.length()] = '\0';
  FILE* fCfg = fopen(filename, "r");
  
  if(initFromParamFile(fCfg))
  {
    delete filename;
    fclose(fCfg);
    delete fCfg;
    return true;
  }
  else
  {
    delete filename;
    fclose(fCfg);
    delete fCfg;
    return false;
  }
}

bool SBPLArmPlannerParams::initMotionPrimsFromFile(FILE* fCfg)
{
  char sTemp[1024];
  int nrows=0,ncols=0, short_mprims=0;


  if(fCfg == NULL)
  {
    ROS_ERROR("ERROR: unable to open the params file. Exiting.");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1."); 
  if(strcmp(sTemp, "Motion_Primitives(degrees):") != 0)
  {
    ROS_ERROR("ERROR: First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
    return false;
  }

  //number of actions
  if(fscanf(fCfg,"%s",sTemp) < 1) 
  {
    ROS_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    nrows = atoi(sTemp);
  
  //length of joint array
  if(fscanf(fCfg,"%s",sTemp) < 1)
  {
    ROS_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    ncols = atoi(sTemp);

  //number of short distance motion primitives
  if(fscanf(fCfg,"%s",sTemp) < 1)
  { 
    ROS_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    short_mprims = atoi(sTemp);

  if(short_mprims == nrows)
    ROS_ERROR("Error: # of motion prims == # of short distance motion prims. No long distance motion prims set.");

  std::vector<double> mprim(ncols,0);

  for (int i=0; i < nrows; ++i)
  {
    for(int j=0; j < ncols; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_WARN("Parsed string has length < 1.");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        mprim[j] = atof(sTemp);
      else
      {
        ROS_ERROR("ERROR: End of parameter file reached prematurely. Check for newline.");
        return false;
      }
    }
    if(i < (nrows-short_mprims))
      addMotionPrim(mprim,true,false);
    else
      addMotionPrim(mprim,true,true);
  }
 
  max_mprim_offset_ = getLargestMotionPrimOffset(); 

  return true;
}

bool SBPLArmPlannerParams::initLongMotionPrimsFromFile(FILE* fCfg)
{
  char sTemp[1024];
  int ndof=0;
  MotionPrimitive temp;
  std::vector<double> p;
  std::vector<bool> add_reflect(6,0);
  std::vector<std::string> adaptive_mprims;

  if(fCfg == NULL)
  {
    ROS_ERROR("Failed to open the motion primitive file. Exiting.");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    ROS_INFO("Parsed string has length < 1.\n");

  while(!feof(fCfg))
  {
    if(sTemp[0] == '#') //comments
    {
      if(fgets(sTemp, 1024,fCfg) == NULL)
        ROS_WARN("Failed to parse comment.");
    }
    else if(strcmp(sTemp, "degrees_of_freedom:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Failed to parse # degrees of freedom."); 
      ndof = atoi(sTemp);
    }
    else if(strcmp(sTemp, "xyz_resolution(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Failed to parse xyz resolution."); 
      xyz_resolution_ = atof(sTemp);
      if(xyz_resolution_ != resolution_)
        ROS_ERROR("Resolution in motion primitive file does not match the resolution of the occupancy grid.");
    }
    else if(strcmp(sTemp, "adaptive_motion_primitive:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Failed to parse adaptive_motion_primitive name."); 
      adaptive_mprims.push_back(sTemp);
    }
    else if(strcmp(sTemp, "rpy_resolution(degrees):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Failed to parse rpy resolution."); 
      rpy_resolution_ = atof(sTemp) * M_PI/180.0;
    }
    else if(strcmp(sTemp, "free_angle_resolution(degrees):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Failed to parse free angle resolution."); 
      fa_resolution_ = atof(sTemp) * M_PI/180.0;
    }
    //type needs to be defined first
    else if(strcmp(sTemp, "type:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Parsed string has length < 1. (type)"); 
 
      if(strcmp(sTemp,"long_distance") == 0)
        temp.type = LONG_DISTANCE;   
      else if(strcmp(sTemp,"short_distance") == 0)
        temp.type = SHORT_DISTANCE;   
      else
      {
        ROS_ERROR("Invalid primitive type. Currently only two types of motion primitives - 'short_distance' or 'long_distance'. Assuming long_distance.");
        temp.type = 0;
      }

      add_reflect.resize(6,0);
      temp.m.clear();
      temp.group = -1;
      temp.coord.resize(ndof,0);
    }
    else if(strcmp(sTemp, "group:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Parsed string has length < 1. (group)");
      temp.group = atoi(sTemp);
    }
    else if(strcmp(sTemp, "id:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Parsed string has length < 1. (id)");
      temp.id = atoi(sTemp);
    }
    else if(strcmp(sTemp, "end_coords(cells):") == 0)
    {
      for(int k = 0; k < ndof; ++k)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          ROS_WARN("Parsed string has length < 1. (coords)");
        temp.coord[k] = atoi(sTemp);
      }
    }
    //add same primitive flipped around a certain axis
    else if(strcmp(sTemp, "add_reflect(no/x/y/z/xy/xz/yz):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Parsed string has length < 1. (add_reflect)");
    
      if(strcmp(sTemp,"x") == 0)
        add_reflect[0] = true;    
      else if(strcmp(sTemp,"y") == 0)
        add_reflect[1] = true;    
      else if(strcmp(sTemp,"z") == 0)
        add_reflect[2] = true;    
      else if(strcmp(sTemp,"xy") == 0)
        add_reflect[3] = true;    
      else if(strcmp(sTemp,"xz") == 0)
        add_reflect[4] = true;    
      else if(strcmp(sTemp,"yz") == 0)
        add_reflect[5] = true;    
      else  
        ROS_ERROR("Not adding inverse.Either invalid axis or 'no' selected.");
    }
    //add same primitive rotated around a certain axis n times
    /*
    else if(strcmp(sTemp, "evenly_distribute_about_x:") == 0)
    {
      if(fscanf(fCfg,"%d",n_rotate_x) < 1)
        ROS_WARN("Failed to parse # of evenly distributed primitives about x-axis.");
    }
    else if(strcmp(sTemp, "evenly_distribute_about_y:") == 0)
    {
      if(fscanf(fCfg,"%d",n_rotate_y) < 1)
        ROS_WARN("Failed to parse # of evenly distributed primitives about y-axis.");
    }
    else if(strcmp(sTemp, "evenly_distribute_about_z:") == 0)
    {
      if(fscanf(fCfg,"%d",n_rotate_z) < 1)
        ROS_WARN("Failed to parse # of evenly distributed primitives about z-axis.");
    }
    */
    //parse the primitive
    else if(strcmp(sTemp, "intermediate_steps:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_WARN("Parsed string has length < 1. (intermediate_steps)"); 
    
      temp.nsteps = atoi(sTemp);

      for(int i=0; i < temp.nsteps; ++i)
      {
        p.resize(ndof,0);

        for(int j=0; j < ndof; ++j)
        {
          if(fscanf(fCfg,"%s",sTemp) < 1)
            ROS_WARN("Expected %d values in motion primitive. Adding zeroes instead.", ndof);

          p[j] = atof(sTemp);  
        }
        temp.m.push_back(p);
      }
      mp_.push_back(temp);
    }
    else
      ROS_WARN("Found unexpected line in motion primitive file. (%s)", sTemp);

    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("Parsed string has length < 1. (%s)", sTemp);
  }

  // add an orientation solving motion primitive
  temp.nsteps = 1;
  temp.group = 0;
  temp.coord.clear();   //hack
  temp.coord.resize(ndof,0);
  p.clear();
  p.resize(ndof,0);
  temp.m.clear();
  temp.m.push_back(p);

  printf(" WE GOT %d ADAPTIVE MPRIMS", int(adaptive_mprims.size())); fflush(stdout);
  for(size_t i = 0; i < adaptive_mprims.size(); ++i)
  {
    // find the type number that matches the name
    temp.type = -1;
    for(size_t j = 0; j < NUMBER_OF_MPRIM_TYPES; ++j)
    {
      if(adaptive_mprims[i].compare(motion_primitive_type_names_[j]) == 0)
      {
        temp.type = j;
        break;
      }
    }
    if(temp.type == -1)
    {
      ROS_ERROR("[prms] Unrecognized adaptive motion primitive name (%s). Exiting.", adaptive_mprims[i].c_str());
      return false;
    }
    // add it to mprim vector
    mp_.push_back(temp);
  }

  printLongMotionPrims("cartesian_mprims");
  return true;
}

bool SBPLArmPlannerParams::initFromParamFile(FILE* fCfg)
{ 
  char sTemp[1024];
  int nrows=0,ncols=0, short_mprims=0;

  if(fCfg == NULL)
  {
    ROS_ERROR("ERROR: unable to open the params file. Exiting.\n");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    ROS_INFO("Parsed string has length < 1.\n");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "environment_size(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      sizeX_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      sizeY_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      sizeZ_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "environment_origin(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      originX_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      originY_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      originZ_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "resolution(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("Parsed string has length < 1.\n");
      resolution_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "use_bfs_heuristic:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_bfs_heuristic_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_orientation_solver:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_orientation_solver_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_ik:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_ik_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "sum_heuristics:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      sum_heuristics_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_research_heuristic:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_research_heuristic_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "plan_to_6d_pose_constraint:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_6d_pose_goal_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"planner_epsilon:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      epsilon_ = atof(sTemp);
    }
    else if(strcmp(sTemp,"use_multiresolution_motion_primitives:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_multires_mprims_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"use_uniform_obstacle_cost:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      use_uniform_cost_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"short_distance_mprims_threshold(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      short_dist_mprims_thresh_m_ = atof(sTemp);
    }
    else if(strcmp(sTemp,"check_if_at_goal_function(0:IK,1:OP):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      is_goal_function_ = atoi(sTemp);
    }   
    else if(strcmp(sTemp,"two_calls_to_orientation_planner:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.\n");
      two_calls_to_op_ = atoi(sTemp);
    }   
    else if(strcmp(sTemp,"verbose:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.");
      verbose_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"solve_for_ik_threshold(distance):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_INFO("Parsed string has length < 1.");
      solve_for_ik_thresh_m_ = atof(sTemp);
    }
    //motion primitives must be last thing in the file

    else if(strcmp(sTemp, "Motion_Primitives(degrees):") == 0)
    {
      break;
    }

    else
    {
      ROS_INFO("Error: Invalid Field name (%s) in parameter file.",sTemp);
      //return false;
    }
    if(fscanf(fCfg,"%s",sTemp) < 1) 
      ROS_INFO("Parsed string has length < 1.");
  }


  //number of actions
  if(fscanf(fCfg,"%s",sTemp) < 1) 
  {
    ROS_INFO("Parsed string has length < 1.");
    return false;
  }
  else
    nrows = atoi(sTemp);
  
  //length of joint array
  if(fscanf(fCfg,"%s",sTemp) < 1)
  {
    ROS_INFO("Parsed string has length < 1.");
    return false;
  }
  else
    ncols = atoi(sTemp);

  //number of short distance motion primitives
  if(fscanf(fCfg,"%s",sTemp) < 1)
  { 
    ROS_INFO("Parsed string has length < 1.");
    return false;
  }
  else
    short_mprims = atoi(sTemp);

  std::vector<double> mprim(ncols,0);

  for (int i=0; i < nrows; ++i)
  {
    for(int j=0; j < ncols; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        ROS_WARN("Parsed string has length < 1.");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        mprim[j] = atof(sTemp);
      else
      {
        ROS_ERROR("ERROR: End of parameter file reached prematurely. Check for newline.");
        return false;
      }
    }
    if(i < (nrows-short_mprims))
      addMotionPrim(mprim,true,false);
    else
      addMotionPrim(mprim,true,true);
  }


  short_dist_mprims_thresh_c_ = short_dist_mprims_thresh_m_ / resolution_;

  solve_for_ik_thresh_ = (solve_for_ik_thresh_m_ /resolution_) * cost_per_cell_;

  max_mprim_offset_ = getLargestMotionPrimOffset(); 
    
  ROS_INFO("Successfully parsed parameters file");
  return true;
}

void SBPLArmPlannerParams::setCellCost(int cost_per_cell)
{
  cost_per_cell_ = cost_per_cell;
  solve_for_ik_thresh_ = (solve_for_ik_thresh_m_ / resolution_) * cost_per_cell_;
  short_dist_mprims_thresh_c_ = short_dist_mprims_thresh_m_/resolution_ * cost_per_cell;
}

void SBPLArmPlannerParams::addMotionPrim(std::vector<double> mprim, bool add_converse, bool short_dist_mprim)
{
  if(short_dist_mprim)
  {
    mprims_.push_back(mprim);
    num_short_dist_mprims_++;
    if(add_converse)
    {
      for(int i = 0; i < int(mprim.size()); ++i)
      {
        if(mprim[i] != 0)
          mprim[i] *=  -1;
      }
      mprims_.push_back(mprim);
      num_short_dist_mprims_++;
    }
  }
  else
  {
    std::vector<std::vector<double> >::iterator it_long_dist;
    it_long_dist = mprims_.begin();
    if(num_long_dist_mprims_ > 1)
    {
      advance(it_long_dist,num_long_dist_mprims_);
      mprims_.insert(it_long_dist, mprim);
    }
    else if(it_long_dist == mprims_.end() || mprims_.empty())
      mprims_.push_back(mprim);

    num_long_dist_mprims_++;
    if(add_converse)
    {
      for(int i = 0; i < int(mprim.size()); ++i)
      {
        if(mprim[i] != 0)
          mprim[i] *=  -1;
      }

      it_long_dist=mprims_.begin();
      advance(it_long_dist,num_long_dist_mprims_);
      mprims_.insert(it_long_dist,mprim);
      num_long_dist_mprims_++;
    } 
  }
  num_mprims_ = num_short_dist_mprims_ + num_long_dist_mprims_;
}

void SBPLArmPlannerParams::printMotionPrims(std::string stream)
{
  int i;
  ROS_DEBUG_NAMED(stream,"Long Distance Motion Primitives: %d", num_long_dist_mprims_);
  for(i = 0; i < num_long_dist_mprims_; ++i)
  {
    ROS_DEBUG_NAMED(stream,"%2d: ",i);
    for(int j = 0; j < int(mprims_[i].size()); ++j)
      ROS_DEBUG_NAMED(stream,"%4.1f ",mprims_[i][j]);
    ROS_DEBUG_NAMED(stream," ");
  }

  ROS_DEBUG_NAMED(stream,"Short Distance Motion Primitives: %d", num_short_dist_mprims_);
  for(; i < num_mprims_; ++i)
  {
    ROS_DEBUG_NAMED(stream,"%2d: ",i);
    for(int j = 0; j < int(mprims_[i].size()); ++j)
      ROS_DEBUG_NAMED(stream,"%4.1f ",mprims_[i][j]);
    ROS_DEBUG_NAMED(stream," ");
  }
}

void SBPLArmPlannerParams::printLongMotionPrims(std::string stream)
{
  ROS_INFO_NAMED(stream, "Statespace Resolution: xyz: %0.3f rpy: %0.3f fa: %0.3f",xyz_resolution_, rpy_resolution_,fa_resolution_);
  ROS_INFO_NAMED(stream, "# Motion Primitives: %d", int(mp_.size()));
  for(int i = 0; i < int(mp_.size()); ++i)
  {
    if(mp_[i].type == LONG_DISTANCE)
      ROS_INFO_NAMED(stream,"[%d] type: long_distance    nsteps: %d group: %d", i, mp_[i].nsteps, mp_[i].group);
    else if(mp_[i].type == SHORT_DISTANCE)  
      ROS_INFO_NAMED(stream,"[%d] type: short_distance   nsteps: %d group: %d", i, mp_[i].nsteps, mp_[i].group);
    else
      ROS_INFO_NAMED(stream,"[%d] type: adaptive   nsteps: %d group: %d", i, mp_[i].nsteps, mp_[i].group);

    for(int j = 0; j < int(mp_[i].m.size()); ++j)
      ROS_INFO_NAMED(stream,"%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", mp_[i].m[j][0],mp_[i].m[j][1],mp_[i].m[j][2],mp_[i].m[j][3],mp_[i].m[j][4],mp_[i].m[j][5],mp_[i].m[j][6]);

    ROS_INFO_NAMED(stream," ");
  }
}

void SBPLArmPlannerParams::printParams(std::string stream)
{
  ROS_DEBUG_NAMED(stream,"Arm Planner Parameters:");
  ROS_DEBUG_NAMED(stream,"%40s: %d", "# motion primitives",num_mprims_);
  ROS_DEBUG_NAMED(stream,"%40s: %d", "# short distance motion primitives", num_short_dist_mprims_);
  ROS_DEBUG_NAMED(stream,"%40s: %d", "# long distance motion primitives", num_long_dist_mprims_);
  ROS_DEBUG_NAMED(stream,"%40s: %.2f", "epsilon",epsilon_);
  ROS_DEBUG_NAMED(stream,"%40s: %s", "use multi-resolution motion primitives", use_multires_mprims_ ? "yes" : "no");
  ROS_DEBUG_NAMED(stream,"%40s: %s", "use dijkstra heuristic", use_bfs_heuristic_ ? "yes" : "no");
  ROS_DEBUG_NAMED(stream,"%40s: %s", "use research heuristic", use_research_heuristic_ ? "yes" : "no");
  ROS_DEBUG_NAMED(stream,"%40s: %s", "h = h_elbow + h_endeff", sum_heuristics_ ? "yes" : "no"); 
  ROS_DEBUG_NAMED(stream,"%40s: %s", "use a uniform cost",use_uniform_cost_ ? "yes" : "no");
  ROS_DEBUG_NAMED(stream,"%40s: %d", "cost per cell", cost_per_cell_);
  ROS_DEBUG_NAMED(stream,"%40s: %.5f", "distance from goal to start using IK:",solve_for_ik_thresh_m_);
  ROS_DEBUG_NAMED(stream,"%40s: %d", "cost from goal to start using IK:",solve_for_ik_thresh_);
  ROS_DEBUG_NAMED(stream," ");
}

double SBPLArmPlannerParams::getSmallestShoulderPanMotion()
{
  if(environment_type_.compare("cartesian") == 0)
    return 0.0698; // 4 deg

  double min_pan = 360.0;
  for (int i = 0; i < num_mprims_; i++)
  {
    if(mprims_[i][0] > 0 && mprims_[i][0] < min_pan) 
      min_pan = mprims_[i][0];
  }

  min_pan = angles::normalize_angle(angles::from_degrees(min_pan));
  ROS_INFO("Smallest shoulder pan motion is %0.3f rad.\n", min_pan);
  
  return min_pan;
}

double SBPLArmPlannerParams::getLargestMotionPrimOffset()
{
  double max_offset = 0;
  for (int i = 0; i < num_mprims_; i++)
  {
    for(size_t j = 0; j < mprims_[i].size(); j++)
    {
      if(fabs(mprims_[i][j]) > max_offset)
        max_offset = fabs(mprims_[i][j]);
    }
  }

  max_offset = angles::normalize_angle(angles::from_degrees(max_offset));
  ROS_INFO("Largest single joint offset in all Motion Prims is %0.3f rad.", max_offset);
  
  return max_offset;
}

bool SBPLArmPlannerParams::initMotionPrims(FILE* fCfg)
{
  if(environment_type_.compare("jointspace") == 0)
    return initMotionPrimsFromFile(fCfg);
  else if(environment_type_.compare("cartesian") == 0)
    return initLongMotionPrimsFromFile(fCfg);
  else
  {
    ROS_ERROR("[prms] Cannot initialize motion primitives from file because environment type is not set.");
    return false;
  }
}

void SBPLArmPlannerParams::printMotionPrim(MotionPrimitive mp, std::string text)
{
  ROS_INFO("Motion Primitive: %s", text.c_str());
  ROS_INFO("type: %20s  id: %d  nsteps: %d  group: %d", motion_primitive_type_names_[int(mp.type)].c_str(), mp.id, mp.nsteps, mp.group);
  ROS_INFO("coord: %2d %2d %2d %2d %2d %2d %d", mp.coord[0], mp.coord[1], mp.coord[2], mp.coord[3], mp.coord[4], mp.coord[5], mp.coord[6]);
  for(int j = 0; j < int(mp.m.size()); ++j)
    ROS_INFO("[waypoint: %d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", j, mp.m[j][0],mp.m[j][1],mp.m[j][2],mp.m[j][3],mp.m[j][4],mp.m[j][5],mp.m[j][6]);
  ROS_INFO(" ");
}

void SBPLArmPlannerParams::changeLoggerLevel(std::string name, std::string level)
{
  ROSCONSOLE_AUTOINIT;

  //std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;
  std::string logger_name = name;

  ROS_INFO("Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ROS_DEBUG_NAMED(name, "This is a debug statement, and should print if you enabled debug.");
}


}
