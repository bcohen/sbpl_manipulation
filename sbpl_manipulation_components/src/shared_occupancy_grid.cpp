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

#include <sbpl_manipulation_components/shared_occupancy_grid.h>
#include <ros/console.h>
#include <ros/time.h>

using namespace std;

namespace sbpl_arm_planner
{

SharedOccupancyGrid::SharedOccupancyGrid(double dim_x, double dim_y, double dim_z, double resolution, double origin_x, double origin_y, double origin_z) : OccupancyGrid(dim_x, dim_y, dim_z, resolution, origin_x, origin_y, origin_z)
{
  grid_ = new distance_field::PropagationDistanceField(dim_x, dim_y, dim_z, resolution, origin_x, origin_y,  origin_z, 0.40);
  grid_->reset();
  delete_grid_ = true;
  use_shared_grid_ = false;
}

void SharedOccupancyGrid::getGridSize(int &dim_x, int &dim_y, int &dim_z)
{
  if(use_shared_grid_)
  {
    dim_x = num_cells_[0];
    dim_y = num_cells_[1];
    dim_z = num_cells_[2];
  }
  else
  {
    dim_x = grid_->getSize(distance_field::PropagationDistanceField::DIM_X) / grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
    dim_y = grid_->getSize(distance_field::PropagationDistanceField::DIM_Y) / grid_->getResolution(distance_field::PropagationDistanceField::DIM_Y);
    dim_z = grid_->getSize(distance_field::PropagationDistanceField::DIM_Z) / grid_->getResolution(distance_field::PropagationDistanceField::DIM_Z);
  }
}

void SharedOccupancyGrid::getWorldSize(double &dim_x, double &dim_y, double &dim_z)
{
  if(use_shared_grid_)
  {
    dim_x = size_[0];
    dim_y = size_[1];
    dim_z = size_[2];
  }
  else
  {
    dim_x = grid_->getSize(distance_field::PropagationDistanceField::DIM_X);
    dim_y = grid_->getSize(distance_field::PropagationDistanceField::DIM_Y);
    dim_z = grid_->getSize(distance_field::PropagationDistanceField::DIM_Z);
  }
}

void SharedOccupancyGrid::getOrigin(double &wx, double &wy, double &wz)
{
  if(use_shared_grid_)
  {
    wx = origin_[0];
    wy = origin_[1];
    wz = origin_[2];
  }
  else
    grid_->gridToWorld(0, 0, 0, wx, wy, wz);
}

double SharedOccupancyGrid::getResolution()
{
  if(use_shared_grid_)
    return resolution_[0];
  else
    return grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
}

/* For storing Distance Field data in Shared Memory */
bool SharedOccupancyGrid::initSharedDistanceField()
{
  // copy distance field
  copyDistanceField(shared_grid_);

  // store details of distance field
  resolution_[0] = grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
  resolution_[1] = grid_->getResolution(distance_field::PropagationDistanceField::DIM_Y);
  resolution_[2] = grid_->getResolution(distance_field::PropagationDistanceField::DIM_Z);
  getWorldSize(size_[0], size_[1], size_[2]);
  getOrigin(origin_[0], origin_[1], origin_[2]);
  getGridSize(num_cells_[0], num_cells_[1], num_cells_[2]);
  
  // delete distance_field object
  if(delete_grid_)
  {
    delete grid_;
    grid_ = NULL;
  }

  use_shared_grid_ = true;
  return true;
}

SharedDistanceFieldPtr SharedOccupancyGrid::getSharedDistanceFieldPtr()
{
  return &shared_grid_;
}

void SharedOccupancyGrid::copyDistanceField(SharedDistanceField &df)
{
  ros::Time t_start = ros::Time::now();

  int dimx,dimy,dimz;
  getGridSize(dimx, dimy, dimz);
  df.resize(dimx, std::vector<std::vector<double> > (dimy, std::vector<double> (dimz)));

  for(int x = 0; x < dimx; ++x)
  {
    for(int y = 0; y < dimy; ++y)
    {
      for(int z = 0; z < dimz; ++z)
        df[x][y][z] = getDistance(x, y, z); 
    }
  }

  double t_end = (ros::Time::now()-t_start).toSec();
  ROS_WARN("[grid] Copying the distance field took %0.3f seconds.", t_end);
}

}
