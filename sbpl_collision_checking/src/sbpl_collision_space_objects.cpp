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

namespace sbpl_arm_planner
{

void SBPLCollisionSpace::removeAttachedObject()
{
  object_attached_ = false;
  object_spheres_.clear();
  ROS_DEBUG("[cspace] Removed attached object.");
}

void SBPLCollisionSpace::attachSphere(std::string name, std::string link, geometry_msgs::Pose pose, double radius)
{
  object_attached_ = true;
  attached_object_frame_ = link;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  object_spheres_.resize(1);
  object_spheres_[0].name = name;
  object_spheres_[0].v.x(pose.position.x);
  object_spheres_[0].v.y(pose.position.y);
  object_spheres_[0].v.z(pose.position.z);
  object_spheres_[0].radius = radius;
  object_spheres_[0].kdl_chain = attached_object_chain_num_;
  object_spheres_[0].kdl_segment = attached_object_segment_num_;

  ROS_DEBUG("[cspace] frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_); 
  ROS_INFO("[cspace] Attached '%s' sphere.  xyz: %0.3f %0.3f %0.3f   radius: %0.3fm", name.c_str(), object_spheres_[0].v.x(), object_spheres_[0].v.y(), object_spheres_[0].v.z(), radius);
}

void SBPLCollisionSpace::attachCylinder(std::string link, geometry_msgs::Pose pose, double radius, double length)
{
  object_attached_ = true;
  attached_object_frame_ = link;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);

  // compute end points of cylinder
  KDL::Frame center;
  tf::PoseMsgToKDL(pose, center);
  //KDL::Vector top(0.0,0.0,length/2.0), bottom(0.0,0.0,-length/2.0);
  KDL::Vector top(center.p), bottom(center.p);
  std::vector<KDL::Vector> points;

  top.data[2] += length/2.0; 
  bottom.data[2] -= length/2.0; 

  // get spheres 
  leatherman::getIntermediatePoints(top, bottom, radius, points);
  object_spheres_.resize(points.size());
  for(size_t i = 0; i < points.size(); ++i)
  {
    object_spheres_[i].name = "attached_" + boost::lexical_cast<std::string>(i);
    object_spheres_[i].v = points[i];
    object_spheres_[i].radius = radius;
    object_spheres_[i].kdl_chain = attached_object_chain_num_;
    object_spheres_[i].kdl_segment = attached_object_segment_num_;
  } 

  ROS_INFO("[cspace] [attached_object] Attaching cylinder. pose: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f spheres: %d", pose.position.x,pose.position.y,pose.position.z, radius, length, int(object_spheres_.size())); 
  ROS_INFO("[cspace] [attached_object]  frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_); 
  ROS_INFO("[cspace] [attached_object]    top: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", top.x(), top.y(), top.z(), radius);
  ROS_INFO("[cspace] [attached_object] bottom: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", bottom.x(), bottom.y(), bottom.z(), radius);
}

void SBPLCollisionSpace::attachCube(std::string name, std::string link, geometry_msgs::Pose pose, double x_dim, double y_dim, double z_dim)
{
  object_attached_ = true;
  std::vector<std::vector<double> > spheres;
  attached_object_frame_  = link;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);

  if(object_spheres_map_.find(name) == object_spheres_map_.end())
  {
    ros::WallTime start = ros::WallTime::now();
    sbpl::SphereEncloser::encloseBox(x_dim, y_dim, z_dim, object_enclosing_sphere_radius_, spheres);
    ROS_DEBUG("[cspace] It took %0.3fsec to enclose %s with %d spheres.", (ros::WallTime::now() - start).toSec(), name.c_str(), int(spheres.size()));
    object_spheres_map_[name] = spheres;
  }
  else
    spheres = object_spheres_map_[name];

  if(spheres.size() <= 3)
    ROS_WARN("[cspace] Attached cube is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached cube. (radius = %0.3fm)", int(spheres.size()), object_enclosing_sphere_radius_);

  // transform spheres
  KDL::Frame center;
  tf::PoseMsgToKDL(pose, center);
  object_spheres_.resize(spheres.size());
  object_spheres_p_.resize(spheres.size());
  for(size_t i = 0; i < spheres.size(); ++i)
  {
    object_spheres_[i].v.x(spheres[i][0]);
    object_spheres_[i].v.y(spheres[i][1]);
    object_spheres_[i].v.z(spheres[i][2]);
    object_spheres_[i].v = center * object_spheres_[i].v;

    object_spheres_[i].name = name + "_" + boost::lexical_cast<std::string>(i);
    object_spheres_[i].radius = object_enclosing_sphere_radius_;
    object_spheres_[i].kdl_chain = attached_object_chain_num_;
    object_spheres_[i].kdl_segment = attached_object_segment_num_;

    object_spheres_p_[i] = &(object_spheres_[i]);
  }

  // low res spheres 
  
  // TODO: Hardcoded for cube
  //sbpl::SphereEncloser::encloseBox(x_dim, y_dim, z_dim, object_enclosing_low_res_sphere_radius_, low_res_spheres);

  std::vector<std::vector<double> > low_res_spheres(5,std::vector<double>(3,0));
  low_res_spheres[0][1] = -0.16;
  low_res_spheres[1][1] = -0.07;
  low_res_spheres[2][1] = -0.00;
  low_res_spheres[3][1] =  0.07;
  low_res_spheres[4][1] =  0.16;
  low_res_object_spheres_.resize(low_res_spheres.size());
  low_res_object_spheres_p_.resize(low_res_spheres.size());
  for(size_t i = 0; i < low_res_spheres.size(); ++i)
  {
    low_res_object_spheres_[i].v.x(low_res_spheres[i][0]);
    low_res_object_spheres_[i].v.y(low_res_spheres[i][1]);
    low_res_object_spheres_[i].v.z(low_res_spheres[i][2]);
    low_res_object_spheres_[i].v = center * low_res_object_spheres_[i].v;

    low_res_object_spheres_[i].name =  name + "_lr_" + boost::lexical_cast<std::string>(i);
    low_res_object_spheres_[i].radius = object_enclosing_low_res_sphere_radius_;
    low_res_object_spheres_[i].kdl_chain = attached_object_chain_num_;
    low_res_object_spheres_[i].kdl_segment = attached_object_segment_num_;

    low_res_object_spheres_p_[i] = &(low_res_object_spheres_[i]);
  }

  att_object_.setSpheres(object_spheres_p_, false);
  att_object_.setSpheres(low_res_object_spheres_p_, true);

  ROS_DEBUG("[cspace] Attaching '%s' represented by %d spheres with dimensions: %0.3f %0.3f %0.3f", name.c_str(), int(spheres.size()), x_dim, y_dim, z_dim);
  ROS_DEBUG("[cspace] ['%s' pose] xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f", name.c_str(), pose.position.x,pose.position.y,pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w); 
}

void SBPLCollisionSpace::attachMesh(std::string name, std::string link, geometry_msgs::Pose pose, const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles)
{
  object_attached_ = true;  
  std::vector<std::vector<double> > spheres;
  attached_object_frame_  = link;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);

  sbpl::SphereEncloser::encloseMesh(vertices, triangles, object_enclosing_sphere_radius_, spheres);
  
  if(spheres.size() <= 3)
    ROS_WARN("[cspace] Attached mesh is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached mesh more accuratly. (radius = %0.3fm)", int(spheres.size()), object_enclosing_sphere_radius_);

  object_spheres_.resize(spheres.size());
  for(size_t i = 0; i < spheres.size(); ++i)
  {
    object_spheres_[i].name = name + "_" + boost::lexical_cast<std::string>(i);
    object_spheres_[i].v.x(spheres[i][0]);
    object_spheres_[i].v.y(spheres[i][1]);
    object_spheres_[i].v.z(spheres[i][2]);
    object_spheres_[i].radius = spheres[i][3];
    object_spheres_[i].kdl_chain = attached_object_chain_num_;
    object_spheres_[i].kdl_segment = attached_object_segment_num_;
  }

  ROS_INFO("[cspace] Attaching '%s' represented by %d spheres with %d vertices and %d triangles.", name.c_str(), int(spheres.size()), int(vertices.size()), int(triangles.size()));
}

bool SBPLCollisionSpace::getAttachedObject(const std::vector<double> &angles, bool low_res, std::vector<std::vector<double> > &xyz)
{
  KDL::Vector v;
  int x,y,z;
  xyz.clear();

  if(!object_attached_)
    return false;

  // compute forward kinematics
  std::vector<std::vector<KDL::Frame> > frames;
  if(!model_.computeDefaultGroupFK(angles, frames))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  if(low_res)
  {
    xyz.resize(low_res_object_spheres_.size(), std::vector<double>(4,0));
    for(size_t i = 0; i < low_res_object_spheres_.size(); ++i)
    {
      v = frames[low_res_object_spheres_[i].kdl_chain][low_res_object_spheres_[i].kdl_segment] * low_res_object_spheres_[i].v;

      // snap to grid
      grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z); 
      grid_->gridToWorld(x, y, z, xyz[i][0], xyz[i][1], xyz[i][2]);

      xyz[i][3] = low_res_object_spheres_[i].radius;
    }
  }
  else
  {
    xyz.resize(object_spheres_.size(), std::vector<double>(4,0));
    for(size_t i = 0; i < object_spheres_.size(); ++i)
    {
      v = frames[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] * object_spheres_[i].v;

      // snap to grid
      grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z); 
      grid_->gridToWorld(x, y, z, xyz[i][0], xyz[i][1], xyz[i][2]);

      xyz[i][3] = object_spheres_[i].radius;
    }

  }
  return true;
}

void SBPLCollisionSpace::processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object)
{
  if(object.id.compare("all") == 0) // ignoring the operation type
  {
    removeAllCollisionObjects();
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    removeCollisionObject(object);
  }
  else
    ROS_ERROR("[cspace] Collision object operation '%d' isn't supported yet.", object.operation.operation);
}

void SBPLCollisionSpace::addCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  object_map_[object.id] = object;

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      std::vector<double> dims(3);
      dims[0] = object.shapes[i].dimensions[0];
      dims[1] = object.shapes[i].dimensions[1];
      dims[2] = object.shapes[i].dimensions[2];
      object_voxel_map_[object.id].clear();
      grid_->getOccupiedVoxels(object.poses[i], dims, object_voxel_map_[object.id]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    {
      std::vector<std::vector<double> > voxels;
      sbpl::Voxelizer::voxelizeSphere(object.shapes[i].dimensions[0], object.poses[i], grid_->getResolution(), voxels, true);
      object_voxel_map_[object.id].clear();
      object_voxel_map_[object.id].resize(voxels.size());

      // transform into the world frame
      Eigen::Affine3d m = Eigen::Affine3d(Eigen::Translation3d(object.poses[i].position.x, object.poses[i].position.y, object.poses[i].position.z)*Eigen::Quaterniond(object.poses[i].orientation.x, object.poses[i].orientation.y, object.poses[i].orientation.z, object.poses[i].orientation.w).toRotationMatrix());
      for(size_t j = 0; j <  voxels.size(); ++j)
      {
        if(voxels[j].size() < 3)
        {
          ROS_ERROR("[cspace] Expected 'voxels' to have length 3.");
          continue;
        }
        object_voxel_map_[object.id][j].x() = (voxels[j][0]);
        object_voxel_map_[object.id][j].y() = (voxels[j][1]);
        object_voxel_map_[object.id][j].z() = (voxels[j][2]);
        object_voxel_map_[object.id][j] = m.rotation() * object_voxel_map_[object.id][j];
        object_voxel_map_[object.id][j] += m.translation();
      }
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      std::vector<std::vector<double> > voxels;
      sbpl::Voxelizer::voxelizeMesh(object.shapes[i].vertices, object.shapes[i].triangles, grid_->getResolution(), voxels, true);
      object_voxel_map_[object.id].clear();
      object_voxel_map_[object.id].resize(voxels.size());
      
      // transform into the world frame
      Eigen::Affine3d m = Eigen::Affine3d(Eigen::Translation3d(object.poses[i].position.x, object.poses[i].position.y, object.poses[i].position.z)*Eigen::Quaterniond(object.poses[i].orientation.x, object.poses[i].orientation.y, object.poses[i].orientation.z, object.poses[i].orientation.w).toRotationMatrix());
      for(size_t j = 0; j <  voxels.size(); ++j)
      {
        if(voxels[j].size() < 3)
        {
          ROS_ERROR("[cspace] Expected 'voxels' to have length 3.");
          continue;
        }
        object_voxel_map_[object.id][j].x() = (voxels[j][0]);
        object_voxel_map_[object.id][j].y() = (voxels[j][1]);
        object_voxel_map_[object.id][j].z() = (voxels[j][2]);
        object_voxel_map_[object.id][j] = m.rotation() * object_voxel_map_[object.id][j];
        object_voxel_map_[object.id][j] += m.translation();
      }
    }
    else
      ROS_WARN("[cspace] Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }

  // add this object to list of objects that get added to grid
  bool new_object = true;
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      ROS_DEBUG("[cspace] Received %s collision object again. Not adding.",object.id.c_str());
      new_object = false;
      break;
    }
  }
  if(new_object)
    known_objects_.push_back(object.id);

  grid_->addPointsToField(object_voxel_map_[object.id]);
}

void SBPLCollisionSpace::removeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      known_objects_.erase(known_objects_.begin() + i);
      ROS_INFO("[cspace] Removing %s from list of known collision objects.", object.id.c_str());
    }
  }
}

void SBPLCollisionSpace::removeAllCollisionObjects()
{
  known_objects_.clear();
}

void SBPLCollisionSpace::putCollisionObjectsInGrid()
{
  ROS_DEBUG("[cspace] Putting %d known objects in grid.", int(known_objects_.size()));
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    grid_->addPointsToField(object_voxel_map_[known_objects_[i]]);
    ROS_DEBUG("[cspace] [%d] Added %s to grid with %d voxels.", int(i), known_objects_[i].c_str(), int(object_voxel_map_[known_objects_[i]].size()));
  }
}

void SBPLCollisionSpace::getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    for(size_t j = 0; j < object_voxel_map_[known_objects_[i]].size(); ++j)
    {
      pose.position.x = object_voxel_map_[known_objects_[i]][j].x();
      pose.position.y = object_voxel_map_[known_objects_[i]][j].y();
      pose.position.z = object_voxel_map_[known_objects_[i]][j].z();
      points.push_back(pose);
    }
  }
}

void SBPLCollisionSpace::attachObject(const arm_navigation_msgs::AttachedCollisionObject &obj)
{
  std::string link_name = obj.link_name;
  arm_navigation_msgs::CollisionObject object(obj.object);

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    /*
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.header = object.header;
    pose_in.header.stamp = ros::Time();
    pose_in.pose = object.poses[i];
    sbpl_arm_planner::transformPose(pscene_, pose_in.pose, pose_out.pose, object.header.frame_id, attached_object_frame_);
    object.poses[i] = pose_out.pose;
    ROS_WARN("[cspace] [attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s (%0.3f %0.3f %0.3f)", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str(), pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);
    */
    if(object.shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    {
      ROS_DEBUG("[cspace] Attaching a '%s' sphere with radius: %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0]);
      attachSphere(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::CYLINDER)
    {
      ROS_DEBUG("[cspace] Attaching a '%s' cylinder with radius: %0.3fm & length %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
      attachCylinder(link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      ROS_DEBUG("[cspace] Attaching a '%s' mesh with %d triangles & %d vertices is NOT supported right now...", object.id.c_str(), int(object.shapes[i].triangles.size()/3), int(object.shapes[i].vertices.size()));
      attachMesh(object.id, link_name, object.poses[i], object.shapes[i].vertices, object.shapes[i].triangles);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      ROS_DEBUG("[cspace] Attaching a '%s' cube with dimensions {%0.3fm x %0.3fm x %0.3fm}.", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
      attachCube(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
    }
    else
      ROS_WARN("[cspace] Currently attaching objects of type '%d' aren't supported.", object.shapes[i].type);
  }
}

bool SBPLCollisionSpace::setAttachedObjects(const std::vector<arm_navigation_msgs::AttachedCollisionObject> &objects)
{
  for(size_t i = 0; i < objects.size(); ++i)
  {
    if(!model_.doesLinkExist(objects[i].link_name, group_name_))
    {
      ROS_WARN("[cspace] This attached object is not intended for the planning joints of the robot. (link: %s)", objects[i].link_name.c_str());
    }
    // add object
    else if(objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
    {
      ROS_DEBUG("[cspace] Received a message to ADD an object (%s) with %d shapes.", objects[i].object.id.c_str(), int(objects[i].object.shapes.size()));
      attachObject(objects[i]);
    }
    // remove object
    else if(objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_DEBUG("[cspace] Removing object (%s) from gripper.", objects[i].object.id.c_str());
      removeAttachedObject();
    }
    else
      ROS_WARN("Received a collision object with an unknown operation");
  }
  return true;
}


}

