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

#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/environment_robarm3d.h>
 
namespace sbpl_arm_planner {

ActionSet::ActionSet()
{
  env_ = NULL;
  use_multires_mprims_ = true;
  use_ik_ = true;
  short_dist_mprims_thresh_m_ = 0.2;
  ik_amp_dist_thresh_m_= 0.20;

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
}

bool ActionSet::init(std::string filename, EnvironmentROBARM3D *env)
{
  env_ = env;

  FILE* file=NULL;
  if((file=fopen(filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("Failed to open action set file.");
    return false;
  }

  return getMotionPrimitivesFromFile(file);
}

bool ActionSet::getMotionPrimitivesFromFile(FILE* fCfg)
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
        mprim[j] = angles::from_degrees(atof(sTemp));
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

  // add amp
  MotionPrimitive m;
  m.type = SNAP_TO_XYZ_RPY;
  m.group = 2;
  m.id =  mp_.size();
  m.action.push_back(mprim);
  mp_.push_back(m);

  // add amp (orientation_solver)
  MotionPrimitive m;
  m.type = SNAP_TO_RPY;
  m.group = 2;
  m.id =  mp_.size();
  m.action.push_back(mprim);
  mp_.push_back(m);

  return true;
}

void ActionSet::addMotionPrim(const std::vector<double> &mprim, bool add_converse, bool short_dist_mprim)
{
  MotionPrimitive m;

  if(short_dist_mprim)
  {
    m.type = SHORT_DISTANCE;
    m.group = 1;
  }
  else
  {
    m.type = LONG_DISTANCE;
    m.group = 0;
  }

  m.id =  mp_.size();
  m.action.push_back(mprim);    
  mp_.push_back(m);

  if(add_converse)
  {
    Action a;
    a.resize(1);
    a[0] = mprim;
    for(int i = 0; i < int(mprim.size()); ++i)
    {
      if(mprim[i] != 0)
        a[0][i] *= -1;
    }
    m.action = a;
    m.id =  mp_.size();
    mp_.push_back(m);
  }
}

void ActionSet::print()
{
  for(size_t i = 0; i < mp_.size(); ++i)
   mp_[i].print(); 
}

bool ActionSet::getActionSet(const RobotState &parent, std::vector<Action> &actions)
{
  std::vector<double> pose;
  if(!env_->getRobotModel()->computePlanningLinkFK(parent, pose))
    return false;

  // get distance to the goal pose
  double d = env_->getDistanceToGoal(pose[0], pose[1], pose[2]);

  Action a;
  for(size_t i = 0; i < mp_.size(); ++i)
  {
    if(getAction(parent, d, mp_[i], a))
      actions.push_back(a);
  }

  if(actions.empty())
    return false;

  return true;
}

bool ActionSet::getAction(const RobotState &parent, double dist_to_goal, MotionPrimitive &mp, Action &action)
{
  if(mp.type == LONG_DISTANCE)
  {
    if(dist_to_goal <= short_dist_mprims_thresh_m_ && use_multires_mprims_)
      return false;

    return applyMotionPrimitive(parent, mp, action);
  }
  else if(mp.type == SHORT_DISTANCE)
  {
    if(dist_to_goal > short_dist_mprims_thresh_m_ && use_multires_mprims_)
      return false;
    
    return applyMotionPrimitive(parent, mp, action);
  }
  else if(mp.type == SNAP_TO_XYZ_RPY)
  {
    if(dist_to_goal > ik_amp_dist_thresh_m_)
    {
      ROS_ERROR("dist_to_goal: %0.3f", dist_to_goal);
      return false;
    }
    action.resize(1);
    std::vector<double> goal = env_->getGoal();
    if(!env_->getRobotModel()->computeIK(goal, parent, action[0]))
    {
      ROS_ERROR("IK Failed. (dist_to_goal: %0.3f)  (goal:   xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)", dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
      return false;
    }
    std::vector<double> p(6,0);
  
    env_->getRobotModel()->computeFK(action[0], "name", p); 

    ROS_ERROR("[ik] goal:  xyz: % 0.3f % 0.3f % 0.3f rpy: % 0.3f % 0.3f % 0.3f", goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
    //ROS_ERROR("[ik]   fk:  xyz: % 0.3f % 0.3f % 0.3f rpy: % 0.3f % 0.3f % 0.3f", p[0], p[1], p[2], p[3], p[4], p[5]);
    //ROS_ERROR("[ik] diff:  xyz: % 0.3f % 0.3f % 0.3f rpy: % 0.3f % 0.3f % 0.3f", fabs(goal[0]-p[0]), fabs(goal[1]-p[1]), fabs(goal[2]-p[2]), fabs(goal[3]-p[3]), fabs(goal[4]-p[4]), fabs(goal[5]-p[5]));
  }
  else
  {
    ROS_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
    return false; 
  }

  return true;
}

bool ActionSet::applyMotionPrimitive(const RobotState &state, MotionPrimitive &mp, Action &action)
{
  action = mp.action;
  for(size_t i = 0; i < action.size(); ++i)
  {
    if(action[i].size() != state.size())
      return false;
    /*
    ROS_INFO("[action_set] state: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", state[0], state[1], state[2], state[3], state[4], state[5], state[6]);
    mp.print();
    */
    for(size_t j = 0; j < action[i].size(); ++j)
      action[i][j] =  angles::normalize_angle(action[i][j] + state[j]);
  }
  return true;
}


}
