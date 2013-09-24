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

/* \author Benjamin Cohen */

#ifndef _SHARED_OCCUPANCY_GRID_
#define _SHARED_OCCUPANCY_GRID_

#include <sbpl_manipulation_components/occupancy_grid.h>

typedef std::vector<std::vector<std::vector<double> > > SharedDistanceField;

typedef std::vector<std::vector<std::vector<double> > >* SharedDistanceFieldPtr;

namespace sbpl_arm_planner{

class SharedOccupancyGrid : public OccupancyGrid
{
  public:
   
    /** 
     * @brief Constructor 
     * @param dimension of grid along X
     * @param dimension of grid along Y
     * @param dimension of grid along Z
     * @param resolution of grid (meters)
     * @param X coordinate of origin (meters)
     * @param Y coordinate of origin (meters)
     * @param Z coordinate of origin (meters)
    */
    SharedOccupancyGrid(double dim_x, double dim_y, double dim_z, double resolution, double origin_x, double origin_y, double origin_z);

    /** @brief destructor */
    ~SharedOccupancyGrid(){};

    /** @brief convert grid cell coords into world coords*/
    inline void gridToWorld(int x, int y, int z, double &wx, double &wy, double &wz);
    
    /** @brief convert world coords into grid cell coords*/
    inline void worldToGrid(double wx, double wy, double wz, int &x, int &y, int &z); 

    /** @brief get the cell's distance to the nearest obstacle in cells*/
    inline unsigned char getCell(int x, int y, int z);

    /** @brief get the cell's distance to the nearest obstacle in meters*/
    inline double getCell(int *xyz);
    
    inline double getDistance(int x, int y, int z);

    /** @brief check if {x,y,z} is in bounds of the grid */
    inline bool isInBounds(int x, int y, int z);
    
    /** @brief get the dimensions of the grid */
    void getGridSize(int &dim_x, int &dim_y, int &dim_z);

    /** @brief get the dimensions of the world (meters)*/
    void getWorldSize(double &dim_x, double &dim_y, double &dim_z);

    /** @brief get the origin of the world (meters)*/
    void getOrigin(double &wx, double &wy, double &wz);

    /** @brief get the resolution of the world (meters)*/
    double getResolution();

    bool initSharedDistanceField();

    SharedDistanceFieldPtr getSharedDistanceFieldPtr();
    
    void copyDistanceField(std::vector<std::vector<std::vector<double> > > &df);

  private:

    bool use_shared_grid_;
    SharedDistanceField shared_grid_;

    double resolution_[3];
    double size_[3];
    double origin_[3];
    int num_cells_[3];

    inline void gridToWorldShared(int x, int y, int z, double &wx, double &wy, double &wz);
    inline void worldToGridShared(double wx, double wy, double wz, int &x, int &y, int &z);
    inline double getDistanceShared(int x, int y, int z);
    inline unsigned char getCellShared(int x, int y, int z);
    inline double getCellShared(int *xyz);
    inline bool isInBoundsShared(int x, int y, int z);
};

inline void SharedOccupancyGrid::gridToWorld(int x, int y, int z, double &wx, double &wy, double &wz)
{
  if(use_shared_grid_)
    gridToWorldShared(x, y, z, wx, wy, wz);
  else
    grid_->gridToWorld(x, y, z, wx, wy, wz); 
}

inline void SharedOccupancyGrid::worldToGrid(double wx, double wy, double wz, int &x, int &y, int &z)
{
  if(use_shared_grid_)
    worldToGridShared(wx, wy, wz, x, y, z);
  else
    grid_->worldToGrid(wx, wy, wz, x, y, z);

  if((x > 10000) || (y > 10000) || (z > 10000) ||
     (x < 0) || (y < 0) || (z < 0))
  {
    ROS_ERROR("[grid] worldToGrid converted %0.5f %0.5f %0.5f to %d %d %d", wx, wy, wz, x, y, z);
    fflush(stdout);
  }
}

inline double SharedOccupancyGrid::getDistance(int x, int y, int z)
{
  if(use_shared_grid_)
    return getDistanceShared(x,y,z);
  else
    return grid_->getDistanceFromCell(x,y,z);
}

inline unsigned char SharedOccupancyGrid::getCell(int x, int y, int z)
{
  if(use_shared_grid_)
    return getCellShared(x,y,z);
  else
    return (unsigned char)(grid_->getDistanceFromCell(x,y,z) / grid_->getResolution(distance_field::PropagationDistanceField::DIM_X));
}

inline double SharedOccupancyGrid::getCell(int *xyz)
{
  if(use_shared_grid_)
    return getCellShared(xyz);
  else
    return grid_->getDistanceFromCell(xyz[0],xyz[1],xyz[2]);
}

inline bool SharedOccupancyGrid::isInBounds(int x, int y, int z)
{
  if(use_shared_grid_)
    return isInBoundsShared(x,y,z);
  else
    return (
      x>=0 && x<grid_->getNumCells(distance_field::PropagationDistanceField::DIM_X) &&
      y>=0 && y<grid_->getNumCells(distance_field::PropagationDistanceField::DIM_Y) &&
      z>=0 && z<grid_->getNumCells(distance_field::PropagationDistanceField::DIM_Z));
}

inline void SharedOccupancyGrid::gridToWorldShared(int x, int y, int z, double &wx, double &wy, double &wz)
{
  wx = origin_[0] + resolution_[0]*(double(x));
  wy = origin_[1] + resolution_[1]*(double(y));
  wz = origin_[2] + resolution_[2]*(double(z));
}

inline void SharedOccupancyGrid::worldToGridShared(double wx, double wy, double wz, int &x, int &y, int &z)
{
  x = (wx-origin_[0])/resolution_[0];
  y = (wy-origin_[1])/resolution_[1];
  z = (wz-origin_[2])/resolution_[2];
  if (x > 0)
    x = floor(x + 0.5);
  else
    x = ceil(x - 0.5);

  if (y > 0)
    y = floor(y + 0.5);
  else
    y = ceil(y - 0.5);

  if (z > 0)
    z = floor(z + 0.5);
  else
    z = ceil(z - 0.5);
}

inline double SharedOccupancyGrid::getDistanceShared(int x, int y, int z)
{
  return shared_grid_[x][y][z];
}

inline unsigned char SharedOccupancyGrid::getCellShared(int x, int y, int z)
{
  return (unsigned char)(getDistanceShared(x,y,z) / resolution_[0]);
}

inline double SharedOccupancyGrid::getCellShared(int *xyz)
{
  return getCellShared(xyz[0],xyz[1],xyz[2]);
}

inline bool SharedOccupancyGrid::isInBoundsShared(int x, int y, int z)
{
  return ( x>=0 && x<num_cells_[0] &&
            y>=0 && y<num_cells_[1] &&
            z>=0 && z<num_cells_[2] );
}

}

#endif
