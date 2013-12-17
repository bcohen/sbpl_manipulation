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
  padding_ = 0.01;
  object_enclosing_sphere_radius_ = 0.03;
}

void SBPLCollisionSpace::setPadding(double padding)
{
  padding_ = padding;
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

  // get the collision spheres for the robot
  model_.getDefaultGroupSpheres(spheres_);

  //model_.printGroups();
  //model_.printDebugInfo(group_name);

  if(!updateVoxelGroups())
    return false;

  return true;
}

bool SBPLCollisionSpace::checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, double &dist)
{
  double dist_temp=100.0;
  dist = 100.0;
  KDL::Vector v;
  int x,y,z;
  Sphere s;
  bool in_collision = false;
  if(visualize)
    collision_spheres_.clear();

  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  // check attached object
  if(object_attached_)
  {
    for(size_t i = 0; i < object_spheres_.size(); ++i)
    {
      v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] * object_spheres_[i].v;

      grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

      // check bounds
      if(!grid_->isInBounds(x, y, z))
      {
        if(verbose)
          ROS_INFO("[cspace] Sphere %d %d %d is out of bounds.", x, y, z);
        return false;
      }

      // check for collision with world
      if((dist_temp = grid_->getDistance(x,y,z)) <= object_spheres_[i].radius)
      {
        dist = dist_temp;

        if(visualize)
        {
          in_collision = true;
          s = *(spheres_[i]);
          s.v = v;
          collision_spheres_.push_back(s);
        }
        else
          return false;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  // check robot model
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;

    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

    // check bounds
    if(!grid_->isInBounds(x, y, z))
    {
      if(verbose)
        ROS_INFO("[cspace] Sphere '%s' with center at {%0.2f %0.2f %0.2f} (%d %d %d) is out of bounds.", spheres_[i]->name.c_str(), v.x(), v.y(), v.z(), x, y, z);
      return false;
    }

    // check for collision with world
    if((dist_temp = grid_->getDistance(x,y,z)) <= (spheres_[i]->radius + padding_))
    {
      dist = dist_temp;
      if(verbose)
        ROS_INFO("    [sphere %d] name: %6s  x: %d y: %d z: %d radius: %0.3fm  dist: %0.3fm  *collision*", int(i), spheres_[i]->name.c_str(), x, y, z, spheres_[i]->radius + padding_, grid_->getDistance(x,y,z));

      if(visualize)
      {
        in_collision = true;
        s = *(spheres_[i]);
        s.v = v;
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

bool SBPLCollisionSpace::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, int &path_length, int &num_checks, double &dist)
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
    path_length = 0;
    ROS_ERROR_ONCE("[cspace] Failed to interpolate the path. It's probably infeasible due to joint limits.");
    ROS_ERROR("[interpolate]  start: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_norm[0], start_norm[1], start_norm[2], start_norm[3], start_norm[4], start_norm[5], start_norm[6]);
    ROS_ERROR("[interpolate]    end: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", end_norm[0], end_norm[1], end_norm[2], end_norm[3], end_norm[4], end_norm[5], end_norm[6]);
    ROS_ERROR("[interpolate]    min: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", min_limits_[0], min_limits_[1], min_limits_[2], min_limits_[3], min_limits_[4], min_limits_[5], min_limits_[6]);
    ROS_ERROR("[interpolate]    max: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", max_limits_[0], max_limits_[1], max_limits_[2], max_limits_[3], max_limits_[4], max_limits_[5], max_limits_[6]);
    return false;
  }

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
        if(!checkCollision(path[j], verbose, false, dist_temp))
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
      if(!checkCollision(path[i], verbose, false, dist_temp))
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

bool SBPLCollisionSpace::getCollisionSpheres(const std::vector<double> &angles, std::vector<std::vector<double> > &spheres)
{
  std::vector<double> xyzr(4,0);
  std::vector<std::vector<double> > object;
  KDL::Vector v;

  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  // robot
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v; 
    xyzr[0] = v.x();
    xyzr[1] = v.y();
    xyzr[2] = v.z();
    xyzr[3] = spheres_[i]->radius;
    ROS_DEBUG("[%d] [robot] xyz: %0.3f %0.3f %0.3f  radius: %0.3f", int(i), xyzr[0], xyzr[1], xyzr[2], xyzr[3]); 
    spheres.push_back(xyzr);
  }

  // attached object
  if(object_attached_)
  {
    getAttachedObject(angles, object);
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
                                         std::vector<std::vector<double> >& path)
{
  return sbpl::Interpolator::interpolatePath(start, end, min_limits_, max_limits_, inc, path);
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

  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  if(num_spheres > int(spheres_.size()))
    num_spheres = spheres_.size();

  for(int i = 0; i < num_spheres; ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;
    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
    dist = grid_->getDistance(x, y, z) - spheres_[i]->radius;

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

bool SBPLCollisionSpace::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int path_length, int num_checks, double &dist)
{
  return checkPathForCollision(angles0, angles1, false, path_length, num_checks, dist);
}

void SBPLCollisionSpace::setRobotState(const arm_navigation_msgs::RobotState &state)
{
  if(state.joint_state.name.size() != state.joint_state.position.size())
    return;

  for(size_t i = 0; i < state.joint_state.name.size(); ++i)
    model_.setJointPosition(state.joint_state.name[i], state.joint_state.position[i]);

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
  for(size_t i = 0; i < scene.attached_collision_objects.size(); ++i)
  {
    if(!model_.doesLinkExist(scene.attached_collision_objects[i].link_name, group_name_))
    {
      ROS_WARN("[cspace] This attached object is not intended for the planning joints of the robot.");
    }
    // add object
    else if(scene.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
    {
      ROS_DEBUG("[cspace] Received a message to ADD an object (%s) with %d shapes.", scene.attached_collision_objects[i].object.id.c_str(), int(scene.attached_collision_objects[i].object.shapes.size()));
      attachObject(scene.attached_collision_objects[i]);
    }
    // remove object
    else if(scene.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_DEBUG("[cspace] Removing object (%s) from gripper.", scene.attached_collision_objects[i].object.id.c_str());
      removeAttachedObject();
    }
    else
      ROS_WARN("Received a collision object with an unknown operation");
  }

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

