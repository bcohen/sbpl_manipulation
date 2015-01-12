/*
 * Copyright (c) 2011, Maxim Likhachev
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

#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <leatherman/viz.h>

namespace sbpl_arm_planner
{

SBPLCollisionSpace::SBPLCollisionSpace(sbpl_arm_planner::OccupancyGrid* grid)
{
  grid_ = grid;
  group_name_ = "";
  object_attached_ = false;
  padding_ = 0.00;
  object_enclosing_sphere_radius_ = 0.03;
  object_enclosing_low_res_sphere_radius_ = 0.065;
  use_multi_level_collision_check_ = false;
  use_ompl_interpolation_ = true;
  num_interpolation_steps_ = 10;
  check_other_groups_against_world_ = true;
  check_default_group_against_world_ = true;
}

void SBPLCollisionSpace::setPadding(double padding)
{
  padding_ = padding;
}

void SBPLCollisionSpace::setParam(std::string name, double value)
{
  if(name.compare("check_other_groups_against_world") == 0)
    check_other_groups_against_world_ = value;
  else if(name.compare("check_default_group_against_world") == 0)
    check_default_group_against_world_ = value;
  else
    ROS_ERROR("[cspace] Unrecognized param name '%s' received.", name.c_str());
}

void SBPLCollisionSpace::setInterpolationParams(bool use_ompl, int num_steps){
  use_ompl_interpolation_ = use_ompl;
  num_interpolation_steps_ = num_steps;
}

bool SBPLCollisionSpace::setPlanningJoints(const std::vector<std::string> &joint_names)
{
  if(group_name_.empty())
  {
    ROS_ERROR("[cspace] Default group name is not set. Please set it before setting planning joints.");
    return false;
  }

  inc_.resize(joint_names.size(),0.0348);
  min_limits_.resize(joint_names.size(), 0.0);
  max_limits_.resize(joint_names.size(), 0.0);
  continuous_.resize(joint_names.size(), false);
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    bool cont = false;
    if(!model_.getJointLimits(group_name_, joint_names[i], min_limits_[i], max_limits_[i], cont))
    {
      ROS_ERROR("[cspace] Failed to retrieve joint limits for %s.", joint_names[i].c_str()); 
      return false;
    }
    continuous_[i] = cont;
  }

  ROS_INFO("[min_limits] %s", leatherman::getString(min_limits_).c_str());
  ROS_INFO("[max_limits] %s", leatherman::getString(max_limits_).c_str());
  ROS_INFO("[continuous] %s", leatherman::getString(continuous_, "yes", "no").c_str());

  // set the order of the planning joints
  model_.setOrderOfJointPositions(joint_names, group_name_);


  //Set up OMPL for interpolation
  ompl::base::RealVectorStateSpace* r5 = new ompl::base::RealVectorStateSpace(5);
  r5->setDimensionName(0,"shoulder_pan");
  r5->setDimensionName(1,"shoulder_lift");
  r5->setDimensionName(2,"upper_arm_roll");
  r5->setDimensionName(3,"elbow_flex");
  r5->setDimensionName(4,"wrist_flex");
  ompl::base::RealVectorBounds bounds(5);
  bounds.setLow(0, min_limits_[0]);
  bounds.setHigh(0, max_limits_[0]);
  bounds.setLow(1, min_limits_[1]);
  bounds.setHigh(1, max_limits_[1]);
  bounds.setLow(2, min_limits_[2]);
  bounds.setHigh(2, max_limits_[2]);
  bounds.setLow(3, min_limits_[3]);
  bounds.setHigh(3, max_limits_[3]);
  bounds.setLow(4, min_limits_[5]);
  bounds.setHigh(4, max_limits_[5]);
  r5->setBounds(bounds);
  ompl::base::StateSpacePtr r5_p(r5);

  ompl::base::StateSpacePtr forearm(new ompl::base::SO2StateSpace());
  ompl::base::StateSpacePtr wrist(new ompl::base::SO2StateSpace());

  omplStateSpace_ = r5_p + forearm + wrist;
  
  si_.reset(new ompl::base::SpaceInformation(omplStateSpace_));
  si_->setStateValidityCheckingResolution(0.00275);  
  si_->setup();

  return true;
}

bool SBPLCollisionSpace::init(std::string group_name, std::string ns)
{
  group_name_ = group_name;

  // initialize the collision model
  if(!model_.init(ns))
  {
    ROS_ERROR("[cspace] The robot's collision model failed to initialize.");
    return false;
  }

  if(!model_.initAllGroups())
  {
    ROS_ERROR("Failed to initialize all groups.");
    return false;
  } 
 
  // choose the group we are planning for
  if(!model_.setDefaultGroup(group_name_))
  {
    ROS_ERROR("Failed to set the default group to '%s'.", group_name_.c_str());
    return false;
  }

  //model_.printGroups();
  //model_.printDebugInfo(group_name);

  if(!updateVoxelGroups())
    return false;

  return true;
}

void SBPLCollisionSpace::setSphereGroupsForCollisionCheck(const std::vector<std::string> &group_names)
{
  model_.setSphereGroupsForCheckCollision(group_names);
}

bool SBPLCollisionSpace::checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, double &dist)
{ 
  if(!use_multi_level_collision_check_)
    return checkCollision(angles, false, verbose, visualize, dist);
  else
  {
    std::vector<std::vector<std::vector<KDL::Frame> > > frames;

    if(checkCollision(angles, frames, true, verbose, visualize, dist))
      return true;
    else
      return checkCollision(angles, frames, false, verbose, visualize, dist);
  }
}

bool SBPLCollisionSpace::checkCollision(const std::vector<double> &angles, std::vector<std::vector<std::vector<KDL::Frame> > > &frames,  bool low_res, bool verbose, bool visualize, double &dist)
{  
  bool in_collision = false;
  double dist_temp=100.0;
  dist = 100.0;
  std::vector<Sphere*> spheres;
  std::vector<KDL::Vector> dg_spheres, g_spheres, obj_spheres;  
  if(visualize)
    collision_spheres_.clear();

  if(low_res)
    ROS_ERROR("[cspace] Checking low_res model! (object_attached: %d  object_spheres: %d)", object_attached_, int(att_object_.getSpheres(low_res).size()));

  // first group, is default group
  std::vector<Group*> sg;
  model_.getSphereGroups(sg);

  // if empty, low_res is probably enabled
  if(frames.empty())
    frames.resize(sg.size());
    
  // compute FK for default group
  if(frames[0].empty())
  {
    if(!model_.computeDefaultGroupFK(angles, frames[0]))
    {
      ROS_ERROR("[cspace] Failed to compute foward kinematics.");
      return false;
    }
  }

  ROS_DEBUG("[cspace] Checking collisions in checkCollision().");

  // check attached object against world
  if(object_attached_)
  {
    if(!checkSpheresAgainstWorld(frames[0], att_object_.getSpheres(low_res), verbose, visualize, obj_spheres, dist_temp))
    {
      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }


  // check default sphere group against world
  if(check_default_group_against_world_)
  {
    if(!checkSpheresAgainstWorld(frames[0], sg[0]->getSpheres(low_res), verbose, visualize, dg_spheres, dist_temp))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }
  else
    fillInSpherePoses(frames[0], sg[0]->getSpheres(low_res), verbose, visualize, dg_spheres, dist_temp);


  // check default sphere group against attached object
  if(object_attached_)
  {
    if(!checkSphereGroupAgainstSphereGroup(sg[0], &att_object_, dg_spheres, obj_spheres, low_res, low_res, verbose, visualize, dist_temp, 3, 100, 0, 1))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }
  

  // check default spheres against themselves
  if(!checkSphereGroupAgainstSphereGroup(sg[0], sg[0], dg_spheres, dg_spheres, low_res, low_res, verbose, visualize, dist_temp, 0, 1, 3, 10))
  {
    if(dist_temp < dist)
      dist = dist_temp;

    if(!visualize)
      return false;
    else
      in_collision = true;
  }
  if(dist_temp < dist)
    dist = dist_temp;

  // check other sphere groups
  std::vector<double> empty_angles;
  for(size_t i = 1; i < sg.size(); ++i)
  {
    // compute FK for group
    if(frames[i].empty())
    {
      if(!sg[i]->computeFK(empty_angles, frames[i]))
      {
        ROS_ERROR("[cspace] Failed to compute FK for sphere group '%s'.", sg[i]->getName().c_str());
        return false;
      }
    }
    
    if(check_other_groups_against_world_)
    {
      // check group against world
      if(!checkSpheresAgainstWorld(frames[i], sg[i]->getSpheres(low_res), verbose, visualize, g_spheres, dist_temp))
      {
        if(dist_temp < dist)
          dist = dist_temp;

        if(!visualize)
          return false;
        else
          in_collision = true;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }
    else
      fillInSpherePoses(frames[i], sg[i]->getSpheres(low_res), verbose, visualize, g_spheres, dist_temp);

    // check group against attached object
    if(object_attached_)
    {
      if(!checkSphereGroupAgainstSphereGroup(&att_object_, sg[i], obj_spheres, g_spheres, low_res, low_res, verbose, visualize, dist_temp, 0, 100, 0, 100))
      {
        if(dist_temp < dist)
          dist = dist_temp;

        if(!visualize)
          return false;
        else
          in_collision = true;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }

    // check group against default group spheres (TODO: change to all sphere groups?)
    if(!checkSphereGroupAgainstSphereGroup(sg[0], sg[i], dg_spheres, g_spheres, low_res, low_res, verbose, visualize, dist_temp, 0, 100, 0, 100))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }

  if(visualize && in_collision)
    return false;

  return true;
}

bool SBPLCollisionSpace::checkCollision(const std::vector<double> &angles, bool low_res, bool verbose, bool visualize, double &dist)
{  
  bool in_collision = false;
  double dist_temp=100.0;
  dist = 100.0;
  std::vector<Sphere*> spheres;
  std::vector<KDL::Vector> dg_spheres, g_spheres, obj_spheres;
  std::vector<std::vector<KDL::Frame> > dframes, frames;
  if(visualize)
    collision_spheres_.clear();

  if(low_res)
    ROS_ERROR("[cspace] Checking low_res model! (object_attached: %d  object_spheres: %d)", object_attached_, int(att_object_.getSpheres(low_res).size()));

  // first group is default group
  std::vector<Group*> sg;
  model_.getSphereGroups(sg);

  // compute FK for default group
  if(!model_.computeDefaultGroupFK(angles, dframes))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }
  
  // check attached object against world
  if(object_attached_)
  {
    if(!checkSpheresAgainstWorld(dframes, att_object_.getSpheres(low_res), verbose, visualize, obj_spheres, dist_temp))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }

  // check default sphere group against world
  if(check_default_group_against_world_)
  {
    if(!checkSpheresAgainstWorld(dframes, model_.getDefaultGroup()->getSpheres(low_res), verbose, visualize, dg_spheres, dist_temp))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }
  else
    fillInSpherePoses(dframes, model_.getDefaultGroup()->getSpheres(low_res), verbose, visualize, dg_spheres, dist_temp);

  // check default sphere group against attached object
  if(object_attached_)
  {
    if(!checkSphereGroupAgainstSphereGroup(sg[0], &att_object_, dg_spheres, obj_spheres, low_res, low_res, verbose, visualize, dist_temp, 3, 100, 0, 1))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }

  // check default spheres against themselves
  if(!checkSphereGroupAgainstSphereGroup(sg[0], sg[0], dg_spheres, dg_spheres, low_res, low_res, verbose, visualize, dist_temp, 0, 1, 3, 100))
  {
    if(dist_temp < dist)
      dist = dist_temp;

    if(!visualize)
      return false;
    else
      in_collision = true;
  }
  if(dist_temp < dist)
    dist = dist_temp;

  // check other sphere groups
  std::vector<double> empty_angles;
  for(size_t i = 1; i < sg.size(); ++i)
  {
    // compute FK for group
    if(!sg[i]->computeFK(empty_angles, frames))
    {
      ROS_ERROR("[cspace] Failed to compute FK for sphere group '%s'.", sg[i]->getName().c_str());
      return false;
    }

    if(check_other_groups_against_world_)
    {
      // check against world
      if(!checkSpheresAgainstWorld(frames, sg[i]->getSpheres(low_res), verbose, visualize, g_spheres, dist_temp))
      {
        if(dist_temp < dist)
          dist = dist_temp;

        if(!visualize)
          return false;
        else
          in_collision = true;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }
    else
      fillInSpherePoses(frames, sg[i]->getSpheres(low_res), verbose, visualize, g_spheres, dist_temp);

    // check group against attached object
    if(object_attached_)
    {
      if(!checkSphereGroupAgainstSphereGroup(&att_object_, sg[i], obj_spheres, g_spheres, low_res, low_res, verbose, visualize, dist_temp, 0, 100, 0, 100))
      {
        if(dist_temp < dist)
          dist = dist_temp;

        if(!visualize)
          return false;
        else
          in_collision = true;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }

    // check against default group spheres (TODO: change to all sphere groups)
    if(!checkSphereGroupAgainstSphereGroup(model_.getDefaultGroup(), sg[i], dg_spheres, g_spheres, low_res, low_res, verbose, visualize, dist_temp, 0, 100, 0, 100))
    {
      if(dist_temp < dist)
        dist = dist_temp;

      if(!visualize)
        return false;
      else
        in_collision = true;
    }
    if(dist_temp < dist)
      dist = dist_temp; 
  }

  if(visualize && in_collision)
    return false;

  return true;
}

bool SBPLCollisionSpace::checkSphereGroupAgainstWorld(const std::vector<double> &angles, Group *group, bool low_res, bool verbose, bool visualize, double &dist)
{
  std::vector<std::vector<KDL::Frame> > frames;
  std::vector<Sphere*> spheres;
  std::vector<KDL::Vector> sph_poses;

  // compute FK for group
  if(!group->computeFK(angles, frames))
  {
    ROS_ERROR("[cspace] Failed to compute FK for sphere group '%s'.", group->getName().c_str());
    return false;
  }
  return checkSpheresAgainstWorld(frames, group->getSpheres(low_res), verbose, visualize, sph_poses, dist); 
}

bool SBPLCollisionSpace::fillInSpherePoses(const std::vector<std::vector<KDL::Frame> > &frames, const std::vector<Sphere*> &spheres, bool verbose, bool visualize, std::vector<KDL::Vector> &sph_poses, double &dist)
{
  sph_poses.resize(spheres.size());

  for(size_t i = 0; i < spheres.size(); ++i)
    sph_poses[i] = frames[spheres[i]->kdl_chain][spheres[i]->kdl_segment] * spheres[i]->v;

  return true;
}

bool SBPLCollisionSpace::checkSpheresAgainstWorld(const std::vector<std::vector<KDL::Frame> > &frames, const std::vector<Sphere*> &spheres, bool verbose, bool visualize, std::vector<KDL::Vector> &sph_poses, double &dist)
{
  double dist_temp=100.0;
  dist = 100.0;
  int x,y,z;
  bool in_collision = false; 
  Sphere s;
  sph_poses.resize(spheres.size());

  if(spheres.size() == 1)
  {
    ROS_ERROR("[cspace] Checking group against world with 1 spheres!");
    verbose = true;
  }

  //ROS_INFO("frames: %d", int(frames.size()));
  for(size_t i = 0; i < spheres.size(); ++i)
  {
    sph_poses[i] = frames[spheres[i]->kdl_chain][spheres[i]->kdl_segment] * spheres[i]->v;

    grid_->worldToGrid(sph_poses[i].x(), sph_poses[i].y(), sph_poses[i].z(), x, y, z);

    // check bounds
    if(!grid_->isInBounds(x, y, z))
    {
      if(verbose)
        ROS_INFO("[cspace] Sphere '%s' with center at {%0.2f %0.2f %0.2f} (%d %d %d) is out of bounds.", spheres[i]->name.c_str(), sph_poses[i].x(), sph_poses[i].y(), sph_poses[i].z(), x, y, z);
      return false;
    }

    // check for collision with world
    if((dist_temp = grid_->getDistance(x,y,z)) <= (spheres[i]->radius + padding_))
    {
      dist = dist_temp;
      if(verbose)
        ROS_INFO("    [sphere: %d] name: %6s  x: %d y: %d z: %d radius: %0.3fm  dist: %0.3fm  *collision*", int(i), spheres[i]->name.c_str(), x, y, z, spheres[i]->radius + padding_, grid_->getDistance(x,y,z));

      if(visualize)
      {
        in_collision = true;
        s = *(spheres[i]);
        s.v = sph_poses[i];
        collision_spheres_.push_back(s);
      }
      else
        return false;
    }

    if(dist_temp < dist)
      dist = dist_temp;
  }

  if(visualize && in_collision)
    return false;

  return true;
}

bool SBPLCollisionSpace::checkSphereGroupAgainstSphereGroup(Group *group1, Group *group2, const std::vector<KDL::Vector> &spheres1, const std::vector<KDL::Vector> &spheres2, bool low_res1, bool low_res2, bool verbose, bool visualize, double &dist, int group1_min_priority, int group1_max_priority, int group2_min_priority, int group2_max_priority)
{
  bool in_collision = false;
  double d;
  dist = 100;
  Sphere s;
  std::vector<Sphere*> gsph1, gsph2;
  group1->getSpheres(gsph1, low_res1);
  group2->getSpheres(gsph2, low_res2);

  if((gsph1.size() != spheres1.size()) || (gsph2.size() != spheres2.size()))
  {
    ROS_ERROR("[cspace] Length of sphere lists received don't match up. (group1: %s {%d, %d}  group2: %s {%d, %d})", group1->getName().c_str(), int(gsph1.size()), int(spheres1.size()), group2->getName().c_str(), int(gsph2.size()), int(spheres2.size()));
    return false;
  }

  for(size_t i = 0; i < spheres1.size(); ++i)
  {
    if((gsph1[i]->priority > group1_max_priority) || (gsph1[i]->priority < group1_min_priority))
      continue;

    for(size_t j = 0; j < spheres2.size(); ++j)
    {
      if((gsph2[j]->priority > group2_max_priority) || (gsph2[j]->priority < group2_min_priority))
        continue;

      d = leatherman::distance(spheres1[i], spheres2[j]);

      if(d <= (gsph1[i]->radius + padding_ +  gsph2[j]->radius + padding_))
      {
        if(d < dist)
          dist = d;

        if(verbose)
          ROS_WARN("[group1: %s  sphere: %s] [group2: %s  sphere: %s] *collision* found. (rad1: %0.3fm  rad2: %0.3fm  dist: %0.3fm)", group1->getName().c_str(), gsph1[i]->name.c_str(), group2->getName().c_str(), gsph2[j]->name.c_str(), gsph1[i]->radius + padding_, gsph2[j]->radius + padding_, d);

        if(visualize)
        {
          in_collision = true;
          s = *(gsph1[i]);
          s.v = spheres1[i];
          collision_spheres_.push_back(s);
          s = *(gsph2[j]);
          s.v = spheres2[j];
          collision_spheres_.push_back(s);
        }
        else
          return false;
      }
      //else
      //  ROS_INFO("[group1: %s  sphere: %s] [group2: %s  sphere: %s] SAFE. (rad1: %0.3fm  rad2: %0.3fm  dist: %0.3fm)", group1->getName().c_str(), gsph1[i]->name.c_str(), group2->getName().c_str(), gsph2[j]->name.c_str(), gsph1[i]->radius + padding_, gsph2[j]->radius + padding_, d);

      if(d < dist)
        dist = d;
    }
  }

  if(visualize && in_collision)
    return false;

  return true;
}

bool SBPLCollisionSpace::updateVoxelGroups()
{
  bool ret = true;
  std::vector<Group*> vg;
  model_.getVoxelGroups(vg);

  for(size_t i = 0; i < vg.size(); ++i)
  {
    if(!updateVoxelGroup(vg[i]))
    {
      ROS_ERROR("Failed to update the '%s' voxel group.", vg[i]->getName().c_str());
      ret = false;
    }
  }
  return ret;
}

bool SBPLCollisionSpace::updateVoxelGroup(std::string name)
{
  Group* g = model_.getGroup(name);
  return updateVoxelGroup(g);
}

bool SBPLCollisionSpace::updateVoxelGroup(Group *g)
{
  KDL::Vector v;
  std::vector<double> angles;
  std::vector<std::vector<KDL::Frame> > frames;
  std::vector<Eigen::Vector3d> pts;
  ROS_DEBUG("Updating voxel group: %s", g->getName().c_str());
  if(!model_.computeGroupFK(angles, g, frames))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics for group '%s'.", g->getName().c_str());
    return false;
  }

  for(size_t i = 0; i < g->links_.size(); ++i)
  {
    Link* l = &(g->links_[i]);
    pts.clear();
    pts.resize(l->voxels_.v.size());

    ROS_DEBUG("Updating Voxel Group %s with %d voxels", g->getName().c_str(), int(l->voxels_.v.size())); 
    for(size_t j = 0; j < l->voxels_.v.size(); ++j)
    {
      v = frames[l->voxels_.kdl_chain][l->voxels_.kdl_segment] * l->voxels_.v[j];
      pts[j].x() = v.x();
      pts[j].y() = v.y();
      pts[j].z() = v.z();
      ROS_DEBUG("[%s] [%d] xyz: %0.2f %0.2f %0.2f", g->getName().c_str(), int(j), pts[j].x(), pts[j].y(), pts[j].z());
    }
    grid_->addPointsToField(pts);
  }
  return true;
}

bool SBPLCollisionSpace::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path_out)
{
  std::vector<std::vector<std::vector<KDL::Frame> > > frames;
  return checkPathForCollision(start, end, frames, verbose, path_length, num_checks, dist, path_out); 
}

bool SBPLCollisionSpace::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, bool verbose, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path_out)
{
  int inc_cc = 5;
  double dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  dist = 100;
  num_checks = 0;

  for(size_t i=0; i < start.size(); ++i)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }

  if(!interpolatePath(start_norm, end_norm, inc_, path))
  {
    ROS_ERROR("[cspace] Failed to interpolate the path. It's probably infeasible due to joint limits (path_length: %d).", int(path.size()));
    ROS_ERROR("[interpolate]  start: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_norm[0], start_norm[1], start_norm[2], start_norm[3], start_norm[4], start_norm[5], start_norm[6]);
    ROS_ERROR("[interpolate]    end: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", end_norm[0], end_norm[1], end_norm[2], end_norm[3], end_norm[4], end_norm[5], end_norm[6]);
    ROS_ERROR("[interpolate]    min: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", min_limits_[0], min_limits_[1], min_limits_[2], min_limits_[3], min_limits_[4], min_limits_[5], min_limits_[6]);
    ROS_ERROR("[interpolate]    max: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", max_limits_[0], max_limits_[1], max_limits_[2], max_limits_[3], max_limits_[4], max_limits_[5], max_limits_[6]);
    path_length = 0;
    return false;
  }
  if(path_out != NULL)
    *path_out = path;

  // for debugging & statistical purposes
  path_length = path.size();

  // try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        num_checks++;
        if(!isStateValid(path[j], frames, verbose, false, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      num_checks++;
      if(!isStateValid(path[i], frames, verbose, false, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

double SBPLCollisionSpace::isValidLineSegment(const std::vector<int> a, const std::vector<int> b, const int radius)
{
  leatherman::bresenham3d_param_t params;
  int nXYZ[3], retvalue = 1;
  double cell_val, min_dist = 100.0;
  leatherman::CELL3V tempcell;
  vector<leatherman::CELL3V>* pTestedCells=NULL;

  //iterate through the points on the segment
  leatherman::get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    leatherman::get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    if(!grid_->isInBounds(nXYZ[0],nXYZ[1],nXYZ[2]))
      return 0;

    cell_val = grid_->getDistance(nXYZ[0],nXYZ[1],nXYZ[2]);
    if(cell_val <= radius)
    {
      if(pTestedCells == NULL)
        return cell_val;   //return 0
      else
        retvalue = 0;
    }

    if(cell_val < min_dist)
      min_dist = cell_val;

    //insert the tested point
    if(pTestedCells)
    {
      if(cell_val <= radius)
        tempcell.bIsObstacle = true;
      else
        tempcell.bIsObstacle = false;
      tempcell.x = nXYZ[0];
      tempcell.y = nXYZ[1];
      tempcell.z = nXYZ[2];
      pTestedCells->push_back(tempcell);
    }
  } while (leatherman::get_next_point3d(&params));

  if(retvalue)
    return min_dist;
  else
    return 0;
}

bool SBPLCollisionSpace::getCollisionSpheres(const std::vector<double> &angles, Group *group, bool low_res, std::vector<std::vector<double> > &spheres)
{
  std::vector<double> xyzr(4,0);
  std::vector<std::vector<double> > object;
  KDL::Vector v;
  std::vector<std::vector<KDL::Frame> > frames;

  // compute foward kinematics
  if(!group->computeFK(angles, frames))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  // get spheres
  std::vector<Sphere*> sph;
  group->getSpheres(sph, low_res);
  for(size_t i = 0; i < sph.size(); ++i)
  {
    v = frames[sph[i]->kdl_chain][sph[i]->kdl_segment] * sph[i]->v; 
    xyzr[0] = v.x();
    xyzr[1] = v.y();
    xyzr[2] = v.z();
    xyzr[3] = sph[i]->radius;
    ROS_DEBUG("[%d] [robot] xyz: %0.3f %0.3f %0.3f  radius: %0.3f", int(i), xyzr[0], xyzr[1], xyzr[2], xyzr[3]); 
    spheres.push_back(xyzr);
  }

  // attached object
  if(object_attached_)
  {
    getAttachedObject(angles, low_res, object);
    for(size_t i = 0; i < object.size(); ++i)
    {
      xyzr[0] = object[i][0];
      xyzr[1] = object[i][1];
      xyzr[2] = object[i][2];
      xyzr[3] = object[i][3];
      ROS_DEBUG("[%d] [attached] xyz: %0.3f %0.3f %0.3f  radius: %0.3f", int(i), xyzr[0], xyzr[1], xyzr[2], xyzr[3]); 
      spheres.push_back(xyzr);
    }
  }
  return true;
}

void SBPLCollisionSpace::setJointPosition(std::string name, double position)
{
  ROS_DEBUG("[cspace] Setting %s with position = %0.3f.", name.c_str(), position);
  model_.setJointPosition(name, position);
}

bool SBPLCollisionSpace::interpolatePath(const std::vector<double>& start,
                                         const std::vector<double>& end,
                                         const std::vector<double>& inc,
                                         std::vector<std::vector<double> >& path, 
                                         double num_interpolation_steps_per_degree)
{
  if(!use_ompl_interpolation_)
    return sbpl::Interpolator::interpolatePath(start, end, min_limits_, max_limits_, inc, path);
  
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> s0(omplStateSpace_);
  (*(s0->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0] = start[0];
  (*(s0->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1] = start[1];
  (*(s0->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2] = start[2];
  (*(s0->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3] = start[3];
  (*(s0->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4] = start[5];
  s0->as<ompl::base::SO2StateSpace::StateType>(1)->value = start[4];
  s0->as<ompl::base::SO2StateSpace::StateType>(2)->value = start[6];

  ompl::base::ScopedState<ompl::base::CompoundStateSpace> s1(omplStateSpace_);
  (*(s1->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0] = end[0];
  (*(s1->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1] = end[1];
  (*(s1->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2] = end[2];
  (*(s1->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3] = end[3];
  (*(s1->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4] = end[5];
  s1->as<ompl::base::SO2StateSpace::StateType>(1)->value = end[4];
  s1->as<ompl::base::SO2StateSpace::StateType>(2)->value = end[6];

  si_->setStateValidityCheckingResolution(0.00275/num_interpolation_steps_per_degree);
  si_->setup();

  ompl::geometric::PathGeometric geo_path(si_,s0.get(),s1.get());
  int before = geo_path.getStateCount();
  geo_path.interpolate();
  int after = geo_path.getStateCount();
  ROS_DEBUG("[cspace][ompl] before: %d  after: %d  (valid_segment_count_factor: %0.3f  max_extent: %0.3f", before, after, si_->getStateSpace()->getValidSegmentCountFactor(), si_->getStateSpace()->getMaximumExtent());
  
  for(unsigned int i=0; i<geo_path.getStateCount(); i++){
    ompl::base::State* state = geo_path.getState(i);
    const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);
    vector<double> p(7,0);
    p[0] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0];
    p[1] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1];
    p[2] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2];
    p[3] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3];
    p[5] = (*(s->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4];
    p[4] = s->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    p[6] = s->as<ompl::base::SO2StateSpace::StateType>(2)->value;
    path.push_back(p);
  }
  return true;
}

bool SBPLCollisionSpace::interpolatePath(const std::vector<double>& start,
                                         const std::vector<double>& end,
                                         std::vector<std::vector<double> >& path)
{
  return sbpl::Interpolator::interpolatePath(start, end, min_limits_, max_limits_, inc_, path);
}

bool SBPLCollisionSpace::getClearance(const std::vector<double> &angles, int num_spheres, double &avg_dist, double &min_dist)
{
  KDL::Vector v;
  int x,y,z;
  double sum = 0, dist = 100;
  min_dist = 100;
  std::vector<std::vector<KDL::Frame> > frames;
  std::vector<Sphere*> spheres = model_.getDefaultGroup()->getSpheres(false);

  if(!model_.computeDefaultGroupFK(angles, frames))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  if(num_spheres > int(spheres.size()))
    num_spheres = spheres.size();

  for(int i = 0; i < int(spheres.size()); ++i)
  {
    v = frames[spheres[i]->kdl_chain][spheres[i]->kdl_segment] * spheres[i]->v;
    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
    dist = grid_->getDistance(x, y, z) - spheres[i]->radius;

    if(min_dist > dist)
      min_dist = dist;
    sum += dist;
  }

  avg_dist = sum / num_spheres;
  ROS_DEBUG("[cspace]  num_spheres: %d  avg_dist: %2.2f   min_dist: %2.2f", num_spheres, avg_dist, min_dist); 
  return true;
}

bool SBPLCollisionSpace::isStateValid(const std::vector<double> &angles, bool verbose, bool visualize, double &dist)
{
  return checkCollision(angles, verbose, visualize, dist);
}

bool SBPLCollisionSpace::isStateValid(const std::vector<double> &angles, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, bool verbose, bool visualize, double &dist)
{
  if(!frames.empty())
    frames[0].clear();

  if(!use_multi_level_collision_check_)
    return checkCollision(angles, false, verbose, visualize, dist);
  else
  {
    if(checkCollision(angles, frames, true, verbose, visualize, dist))
      return true;
    else
      return checkCollision(angles, frames, false, verbose, visualize, dist);
  }
}

bool SBPLCollisionSpace::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path_out)
{
  return checkPathForCollision(angles0, angles1, false, path_length, num_checks, dist, path_out);
}

bool SBPLCollisionSpace::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path_out)
{
  return checkPathForCollision(angles0, angles1, frames, false, path_length, num_checks, dist, path_out);
}

void SBPLCollisionSpace::setRobotState(const arm_navigation_msgs::RobotState &state, bool reset)
{
  if(state.joint_state.name.size() != state.joint_state.position.size())
    return;

  for(size_t i = 0; i < state.joint_state.name.size(); ++i)
    model_.setJointPosition(state.joint_state.name[i], state.joint_state.position[i]);

  // TODO: turn into its own function
  if(reset)
  {
    grid_->reset();
    putCollisionObjectsInGrid();
    updateVoxelGroups();
  }
}

void SBPLCollisionSpace::setRobotState(const arm_navigation_msgs::RobotState &state)
{
  if(state.joint_state.name.size() != state.joint_state.position.size())
    return;

  for(size_t i = 0; i < state.joint_state.name.size(); ++i)
    model_.setJointPosition(state.joint_state.name[i], state.joint_state.position[i]);

  // TODO: turn into its own function
  grid_->reset();
  putCollisionObjectsInGrid();
  updateVoxelGroups();
}

bool SBPLCollisionSpace::setPlanningScene(const arm_navigation_msgs::PlanningScene &scene)
{
  // robot state
  if(scene.robot_state.joint_state.name.size() != scene.robot_state.joint_state.position.size())
    return false;

  for(size_t i = 0; i < scene.robot_state.joint_state.name.size(); ++i)
    model_.setJointPosition(scene.robot_state.joint_state.name[i], scene.robot_state.joint_state.position[i]);

  if(!model_.setModelToWorldTransform(scene.robot_state.multi_dof_joint_state, scene.collision_map.header.frame_id))
  {
    ROS_ERROR("Failed to set the model-to-world transform. The collision model's frame is different from the collision map's frame.");
    return false;
  }

  // reset the distance field (TODO...shouldn't have to reset everytime)
  grid_->reset();

  // collision objects
  for(size_t i = 0; i < scene.collision_objects.size(); ++i)
  {
    object_map_[scene.collision_objects[i].id] = scene.collision_objects[i];
    processCollisionObjectMsg(scene.collision_objects[i]);
  }
  putCollisionObjectsInGrid();

  // attached collision objects
  if(!setAttachedObjects(scene.attached_collision_objects))
    return false;

  // collision map
  if(scene.collision_map.header.frame_id.compare(grid_->getReferenceFrame()) != 0)
    ROS_WARN_ONCE("collision_map_occ is in %s not in %s", scene.collision_map.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());

  if(!scene.collision_map.boxes.empty())
    grid_->updateFromCollisionMap(scene.collision_map);

  // self collision
  updateVoxelGroups();
  return true;
}

}

