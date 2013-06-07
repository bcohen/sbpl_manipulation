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
  solve_for_ik_thresh_m_= 0.20;

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

bool ActionSet::init(FILE* file, EnvironmentROBARM3D &env)
{
  env_ = env;
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
 
  return true;
}

void ActionSet::addMotionPrim(const std::vector<double> &mprim, bool add_converse, bool short_dist_mprim)
{
  MotionPrimitive m;
  if(short_dist_mprim)
  {
    m.type = SHORT_DISTANCE;
    m.id = 0;
    m.group = 1;
    m.action.push_back(mprim);    
  
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
    }
  }
  else
  {
    m.type = LONG_DISTANCE;
    m.id = 1;
    m.group = 0;
    m.action.push_back(mprim);    
  
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
    }  
  }
  mp_.push_back(m);
}

void ActionSet::print()
{
  for(size_t i = 0; i < mp_.size(); ++i)
   mp_[i].print(); 
}


}
