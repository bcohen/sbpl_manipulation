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

visualization_msgs::MarkerArray SBPLCollisionSpace::getVisualization(std::string type)
{
  visualization_msgs::MarkerArray ma;

  if(type.compare("collision_objects") == 0)
  {
    visualization_msgs::MarkerArray ma1;
    for(size_t i = 0; i < known_objects_.size(); ++i)
    {
      if(object_map_.find(known_objects_[i]) != object_map_.end())
      {
        std::vector<double> hue(object_map_[known_objects_[i]].shapes.size(), 200);
        ma1 = viz::getCollisionObjectMarkerArray(object_map_[known_objects_[i]], hue, object_map_[known_objects_[i]].id, 0);
        ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
      }
    }
  }
  else if(type.compare("collisions") == 0)
  {
    std::vector<double> rad(collision_spheres_.size());
    std::vector<std::vector<double> > sph(collision_spheres_.size(), std::vector<double>(3,0));
    for(size_t i = 0; i < collision_spheres_.size(); ++i)
    {
      sph[i][0] = collision_spheres_[i].v.x();
      sph[i][1] = collision_spheres_[i].v.y();
      sph[i][2] = collision_spheres_[i].v.z();
      rad[i] = collision_spheres_[i].radius + padding_ + 0.005;
      ROS_DEBUG("[col-sph %d] name: %s  xyz: %0.3f %0.3f %0.3f  radius: %0.3f", int(i), collision_spheres_[i].name.c_str(), sph[i][0], sph[i][1], sph[i][2], rad[i]);
    } 
    ma = viz::getSpheresMarkerArray(sph, rad, 240, grid_->getReferenceFrame(), "collision_spheres", 0);
    for(size_t j = 0; j < ma.markers.size(); ++j)
      ma.markers[j].color.a = 1.00; 
  }
  else if(type.compare("collision_object_voxels") == 0)
  {
    visualization_msgs::Marker marker;
    std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
    std::vector<double> color(4,1);
    color[2] = 0;
    std::vector<geometry_msgs::Pose> vposes;
    getCollisionObjectVoxelPoses(vposes);

    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = grid_->getReferenceFrame();
    marker.ns = "collision_object_voxels";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.points.resize(vposes.size());
    for(size_t i = 0; i < vposes.size(); ++i)
    {
      marker.points[i].x = vposes[i].position.x;
      marker.points[i].y = vposes[i].position.y;
      marker.points[i].z = vposes[i].position.z;
    }
    ma.markers.push_back(marker);
  }
  else if(type.compare("collision_model") == 0)
  {
    std::vector<double> angles, rad, radi;
    std::vector<std::vector<double> > sph, sphi;
    std::vector<Group*> grps;
    model_.getSphereGroups(grps);

    for(size_t j = 0; j < grps.size(); ++j)
    {
      sphi.clear();
      radi.clear();
      getCollisionSpheres(angles, grps[j], false, sphi);

      if(sphi.empty() || sphi[0].size() < 4)
        return ma;

      radi.resize(sphi.size());
      for(size_t i = 0; i < sphi.size(); ++i)
        radi[i] = sphi[i][3];

      sph.insert(sph.end(), sphi.begin(), sphi.end());
      rad.insert(rad.end(), radi.begin(), radi.end());
    }
    ma = viz::getSpheresMarkerArray(sph, rad, 90, grid_->getReferenceFrame(), "collision_model", 0); 
  }
  else if(type.compare("low_res_collision_model") == 0)
  {
    std::vector<double> angles, rad, radi;
    std::vector<std::vector<double> > sph, sphi;
    std::vector<Group*> grps;
    model_.getSphereGroups(grps);

    for(size_t j = 0; j < grps.size(); ++j)
    {
      sphi.clear();
      radi.clear();
      getCollisionSpheres(angles, grps[j], true, sphi);

      if(sphi.empty() || sphi[0].size() < 4)
        return ma;

      radi.resize(sphi.size());
      for(size_t i = 0; i < sphi.size(); ++i)
        radi[i] = sphi[i][3];

      sph.insert(sph.end(), sphi.begin(), sphi.end());
      rad.insert(rad.end(), radi.begin(), radi.end());
    }
    ma = viz::getSpheresMarkerArray(sph, rad, 100, grid_->getReferenceFrame(), "low_res_collision_model", 0); 
  }
  else if(type.compare("attached_object") == 0)
  {
    std::vector<double> angles, rad;
    std::vector<std::vector<double> > sph;
    if(object_attached_)
      getAttachedObject(angles, false, sph);

    if(sph.empty() || sph[0].size() < 4)
      return ma;

    rad.resize(sph.size());
    for(size_t i = 0; i < sph.size(); ++i)
      rad[i] = sph[i][3];

    ma = viz::getSpheresMarkerArray(sph, rad, 200, grid_->getReferenceFrame(), "attached_object", 0); 
  }
  else if(type.compare("low_res_attached_object") == 0)
  {
    std::vector<double> angles, rad;
    std::vector<std::vector<double> > sph;
    if(object_attached_)
      getAttachedObject(angles, true, sph);

    if(sph.empty() || sph[0].size() < 4)
      return ma;

    rad.resize(sph.size());
    for(size_t i = 0; i < sph.size(); ++i)
      rad[i] = sph[i][3];

    ma = viz::getSpheresMarkerArray(sph, rad, 200, grid_->getReferenceFrame(), "low_res_attached_object", 0); 
  }

  else
    ma = grid_->getVisualization(type);

  return ma;
}

visualization_msgs::MarkerArray SBPLCollisionSpace::getCollisionModelVisualization(const std::vector<double> &angles)
{
  std::vector<double> rad;
  std::vector<std::vector<double> > sph;
  visualization_msgs::MarkerArray ma;

  getCollisionSpheres(angles, model_.getGroup(group_name_), false, sph);

  if(sph.empty() || sph[0].size() < 4)
    return ma;

  rad.resize(sph.size());
  for(size_t i = 0; i < sph.size(); ++i)
    rad[i] = sph[i][3];

  return viz::getSpheresMarkerArray(sph, rad, 90, grid_->getReferenceFrame(), "collision_model", 0); 
}

// Warning: This function is still in development...TODO: Finish this function
visualization_msgs::MarkerArray SBPLCollisionSpace::getMeshModelVisualization(const std::string group_name, const std::vector<double> &angles)
{
  visualization_msgs::MarkerArray ma;
  geometry_msgs::Pose fpose;
  geometry_msgs::PoseStamped lpose, mpose;
  std::string robot_description, mesh_resource;
  std::vector<std::vector<KDL::Frame> > frames;
  Group* g = model_.getGroup(group_name);

  ros::NodeHandle nh;
  if (!nh.getParam("robot_description", robot_description))
  {
    ROS_ERROR("Failed to get robot_description from param server.");
    return ma;
  }

  // compute foward kinematics
  if(!model_.computeGroupFK(angles, g, frames))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return ma;
  }

  // get link mesh_resources
  for(size_t i = 0; i < g->links_.size(); ++i)
  {
    if(!leatherman::getLinkMesh(robot_description, g->links_[i].root_name_, false, mesh_resource, lpose))
    {
      ROS_ERROR("Failed to get mesh for '%s'.", g->links_[i].root_name_.c_str());
      continue;
    }

    ROS_INFO("Got the mesh! (%s)", mesh_resource.c_str());
    // TODO: Has to be a spheres group
    leatherman::msgFromPose(frames[g->links_[i].spheres_[0].kdl_chain][g->links_[i].spheres_[0].kdl_segment], fpose);
    leatherman::multiply(fpose, lpose.pose, mpose.pose);
    mpose.header.frame_id = "base_link"; //getReferenceFrame();
    ma.markers.push_back(viz::getMeshMarker(mpose, mesh_resource, 180, "robot_model", i));
  }
  return ma;
}

}

