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

#include <sbpl_manipulation_components/occupancy_grid.h>

using namespace std;

namespace sbpl_arm_planner
{

OccupancyGrid::OccupancyGrid(double dim_x, double dim_y, double dim_z, double resolution, double origin_x, double origin_y, double origin_z)
{
  grid_resolution_ = resolution;
  prop_distance_ = 0.40;

  grid_ = new distance_field::PropagationDistanceField(dim_x, dim_y, dim_z, grid_resolution_, origin_x, origin_y,  origin_z, prop_distance_);
  grid_->reset();
}

OccupancyGrid::~OccupancyGrid()
{
  delete grid_;
}

void OccupancyGrid::getGridSize(int &dim_x, int &dim_y, int &dim_z)
{
  dim_x = grid_->getSize(distance_field::PropagationDistanceField::DIM_X);
  dim_y = grid_->getSize(distance_field::PropagationDistanceField::DIM_Y);
  dim_z = grid_->getSize(distance_field::PropagationDistanceField::DIM_Z);
}

void OccupancyGrid::getWorldSize(double &dim_x, double &dim_y, double &dim_z)
{
  dim_x = grid_->getSize(distance_field::PropagationDistanceField::DIM_X) *
    grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
  dim_y = grid_->getSize(distance_field::PropagationDistanceField::DIM_Y) *
    grid_->getResolution(distance_field::PropagationDistanceField::DIM_Y);
  dim_z = grid_->getSize(distance_field::PropagationDistanceField::DIM_Z) *
    grid_->getResolution(distance_field::PropagationDistanceField::DIM_Z);
}

void OccupancyGrid::reset()
{
  grid_->reset();
}

void OccupancyGrid::getOrigin(double &wx, double &wy, double &wz)
{
  grid_->gridToWorld(0, 0, 0, wx, wy, wz);
}

double OccupancyGrid::getResolution()
{
  return grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
}

void OccupancyGrid::updateFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  if(collision_map.boxes.empty())
  {
    ROS_DEBUG("[grid] collision map received is empty.");
    return;
  }
  reference_frame_ = collision_map.header.frame_id;
  grid_->addCollisionMapToField(collision_map);
}

void OccupancyGrid::addCollisionCuboid(double origin_x, double origin_y, double origin_z, double size_x, double size_y, double size_z)
{
  int num_points=0;
  std::vector<tf::Vector3> pts;

  for (double x=origin_x-size_x/2.0; x<=origin_x+size_x/2.0; x+=grid_resolution_)
  {
    for (double y=origin_y-size_y/2.0; y<=origin_y+size_y/2.0; y+=grid_resolution_)
    {
      for (double z=origin_z-size_z/2.0; z<=origin_z+size_z/2.0; z+=grid_resolution_)
      {
        pts.push_back(tf::Vector3(x,y,z));
        ++num_points;
      }
    }
  }

  grid_->addPointsToField(pts);
}

void OccupancyGrid::getVoxelsInBox(const geometry_msgs::Pose &pose, const std::vector<double> &dim, std::vector<Eigen::Vector3d> &voxels)
{
  Eigen::Vector3d vin, vout, v(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Matrix3d m(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));

  for (double x=0-dim[0]/2.0; x<=dim[0]/2.0; x+=grid_resolution_)
  {
    for (double y=0-dim[1]/2.0; y<=dim[1]/2.0; y+=grid_resolution_)
    {
      for (double z=0-dim[2]/2.0; z<=dim[2]/2.0; z+=grid_resolution_)
      {
        vin(0) = (x);
        vin(1) = (y);
        vin(2) = (z); 
        vout = m*vin;
        vout += v;

        voxels.push_back(vout);
      }
    }
  }
}


}
