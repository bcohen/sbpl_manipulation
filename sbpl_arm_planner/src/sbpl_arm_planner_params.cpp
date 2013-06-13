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
  use_bfs_heuristic_ = true;
  ready_to_plan_ = false;

  verbose_ = false;
  verbose_heuristics_ = false;
  verbose_collisions_ = false;
  
  cost_multiplier_ = 1000;
  cost_per_cell_ = 1;
  cost_per_meter_ = 0;
  cost_per_second_ = cost_multiplier_;
  time_per_cell_ = 0.05;

  expands_log_ = "expands";
  expands2_log_ = "expands2";
  rmodel_log_ = "arm";
  ik_log_ = "ik";
  cspace_log_ = "cspace";
  solution_log_ = "solution";

  expands_log_level_ = "info";
  expands2_log_level_ = "info";
  ik_log_level_ = "info";
  rmodel_log_level_ = "info";
  cspace_log_level_ = "info";
  solution_log_level_ = "info";
}

bool SBPLArmPlannerParams::init()
{
  ros::NodeHandle nh("~");

  /* planning */
  nh.param("planning/epsilon",epsilon_, 10.0);
  nh.param("planning/use_bfs_heuristic",use_bfs_heuristic_,true);
  nh.param("planning/verbose",verbose_,false);
  nh.param<std::string>("planning/reference_frame",reference_frame_,"");
  nh.param<std::string>("planning/group_name",group_name_,"");

  /* logging */
  nh.param<std::string>("debug/logging/expands", expands_log_level_, "info");
  nh.param<std::string>("debug/logging/expands2", expands2_log_level_, "info");
  nh.param<std::string>("debug/logging/ik", ik_log_level_, "info");
  nh.param<std::string>("debug/logging/robot_model", rmodel_log_level_, "info");
  nh.param<std::string>("debug/logging/collisions", cspace_log_level_, "info");
  nh.param<std::string>("debug/logging/solution", solution_log_level_, "info");

  XmlRpc::XmlRpcValue xlist;
  nh.getParam("planning/planning_joints", xlist);

  if(xlist.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Planning joints list is not an array. Expecting a list with more than one joint.");
    return false;
  }

  // planning joints
  std::string joint_list = std::string(xlist);
  std::stringstream joint_name_stream(joint_list);
  while(joint_name_stream.good() && !joint_name_stream.eof())
  {
    std::string jname;
    joint_name_stream >> jname;
    if(jname.size() == 0)
      continue;
    planning_joints_.push_back(jname);
  }
  num_joints_ = planning_joints_.size();

  // discretization
  std::string p;
  if(nh.hasParam("planning/discretization"))
  {
    nh.getParam("planning/discretization", xlist);
    std::stringstream ss(xlist);
    while(ss >> p)
      coord_vals_.push_back(atof(p.c_str()));

    coord_delta_.resize(coord_vals_.size(),0);
    for(int i = 0; i < num_joints_; ++i)
      coord_delta_[i] = (2.0*M_PI) / coord_vals_[i];
  }
  else
  {
    ROS_ERROR("Discretization of statespace has not been defined.");
    return false;
  }

  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands_log_, expands_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands2_log_, expands2_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + solution_log_, solution_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + ik_log_, ik_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + rmodel_log_, rmodel_log_level_);
  changeLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + cspace_log_, cspace_log_level_);

  return true;
}

void SBPLArmPlannerParams::setCellCost(int cost_per_cell)
{
  cost_per_cell_ = cost_per_cell;
}

void SBPLArmPlannerParams::printParams(std::string stream)
{
  ROS_DEBUG_NAMED(stream,"Manipulation Environment Parameters:");
  ROS_DEBUG_NAMED(stream,"%40s: %.2f", "epsilon",epsilon_);
  ROS_DEBUG_NAMED(stream,"%40s: %s", "use dijkstra heuristic", use_bfs_heuristic_ ? "yes" : "no");
  ROS_DEBUG_NAMED(stream,"%40s: %d", "cost per cell", cost_per_cell_);
  ROS_DEBUG_NAMED(stream,"planning joints: ");
  for(size_t i = 0; i < planning_joints_.size(); ++i)
    ROS_DEBUG_NAMED(stream,"   [%d] %30s", int(i), planning_joints_[i].c_str());
  ROS_DEBUG_NAMED(stream,"discretization: ");
  ROS_DEBUG_NAMED(stream," ");
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
