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

#ifndef _OCCUPANCY_GRID_
#define _OCCUPANCY_GRID_

#include <sys/stat.h>
#include <vector>
#include <fstream>
#include <tf/LinearMath/Vector3.h>
#include <Eigen/Geometry>
#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosbag/bag.h>

/* \brief At this point, this is a very lightweight layer on top of the
 * PropagationDistanceField class. I'll eventually get rid of it once the
 * PDF class has a couple of more things in it.
*/

namespace sbpl_arm_planner{

class OccupancyGrid
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
    OccupancyGrid(double dim_x, double dim_y, double dim_z, double resolution, double origin_x, double origin_y, double origin_z);

    OccupancyGrid(distance_field::PropagationDistanceField* df);

    /** @brief destructor */
    ~OccupancyGrid();

    /** @brief convert grid cell coords into world coords*/
    inline void gridToWorld(int x, int y, int z, double &wx, double &wy, double &wz);
    
    /** @brief convert world coords into grid cell coords*/
    inline void worldToGrid(double wx, double wy, double wz, int &x, int &y, int &z); 

    /** @brief get the cell's distance to the nearest obstacle in cells*/
    inline unsigned char getCell(int x, int y, int z);

    /** @brief get the cell's distance to the nearest obstacle in meters*/
    inline double getCell(int *xyz);
    
    inline double getDistance(int x, int y, int z);
    
    // TODO: Rename this function to match the distance_field API
    inline double getDistanceFromPoint(double x, double y, double z);

    /** @brief check if {x,y,z} is in bounds of the grid */
    inline bool isInBounds(int x, int y, int z);

    /** @brief return a pointer to the distance field */
    inline distance_field::PropagationDistanceField* getDistanceFieldPtr();
    
    /** @brief get the dimensions of the grid */
    void getGridSize(int &dim_x, int &dim_y, int &dim_z);

    /** @brief get the dimensions of the grid */
    void getGridSize(int *dims); //FILL IN THIS FUNCTION

    /** @brief get the dimensions of the world (meters)*/
    void getWorldSize(double &dim_x, double &dim_y, double &dim_z);

    /** @brief get the origin of the world (meters)*/
    void getOrigin(double &wx, double &wy, double &wz);

    /** @brief get the resolution of the world (meters)*/
    double getResolution();

    /** @brief update the distance field from the collision_map */
    void updateFromCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map);

    void updateFromOctree(const octomap::OcTree* oct);

    /** 
     * @brief manually add a cuboid to the collision map
     * @param X_origin_of_cuboid 
     * @param Y_origin_of_cuboid 
     * @param Z_origin_of_cuboid
     * @param size along the X dimension (meters)
     * @param size along the Y dimension (meters)
     * @param size along the Z dimension (meters)
    */
    void addCube(double origin_x, double origin_y, double origin_z, double size_x, double size_y, double size_z);

    void addPointsToField(const std::vector<Eigen::Vector3d> &points);

    void getOccupiedVoxels(std::vector<geometry_msgs::Point> &voxels);

    void getOccupiedVoxels(const geometry_msgs::Pose &pose, const std::vector<double> &dim, std::vector<Eigen::Vector3d> &voxels);

    void getOccupiedVoxels(double x_center, double y_center, double z_center, double radius, std::string text, std::vector<geometry_msgs::Point> &voxels);

    std::string getReferenceFrame();

    void setReferenceFrame(const std::string &frame);

    void reset();

    visualization_msgs::MarkerArray getVisualization(std::string type);

    bool writeOccupancyGridToBagFile(std::string bag_filename, std::string topic_name);

    bool writeCollisionMapToBagFile(const arm_navigation_msgs::CollisionMap &map, std::string bag_filename, std::string topic_name);

  private:

    bool delete_grid_;
    std::string reference_frame_;
    distance_field::PropagationDistanceField* grid_;

    // copied from PropogationDistanceField class in Fuerte
    void addCollisionMapToField(const arm_navigation_msgs::CollisionMap &collision_map);
};

inline distance_field::PropagationDistanceField* OccupancyGrid::getDistanceFieldPtr()
{
  return grid_;
}

inline void OccupancyGrid::gridToWorld(int x, int y, int z, double &wx, double &wy, double &wz)
{
  grid_->gridToWorld(x, y, z, wx, wy, wz); 
}

inline void OccupancyGrid::worldToGrid(double wx, double wy, double wz, int &x, int &y, int &z)
{
  grid_->worldToGrid(wx, wy, wz, x, y, z);

  if((x > 10000) || (y > 10000) || (z > 10000) ||
     (x < 0) || (y < 0) || (z < 0))
  {
    ROS_ERROR("[grid] worldToGrid converted %0.5f %0.5f %0.5f to %d %d %d", wx, wy, wz, x, y, z);
    fflush(stdout);
  }
}

inline double OccupancyGrid::getDistance(int x, int y, int z)
{
  return grid_->getDistance(x,y,z);
}

inline double OccupancyGrid::getDistanceFromPoint(double x, double y, double z)
{
  int gx, gy, gz;
  worldToGrid(x, y, z, gx, gy, gz);
  return grid_->getDistance(gx, gy, gz);
}

inline unsigned char OccupancyGrid::getCell(int x, int y, int z)
{
  return (unsigned char)(grid_->getDistance(x,y,z) / grid_->getResolution());
}

inline double OccupancyGrid::getCell(int *xyz)
{
  return grid_->getDistance(xyz[0],xyz[1],xyz[2]);
}

inline bool OccupancyGrid::isInBounds(int x, int y, int z)
{
  return (
      x>=0 && x<grid_->getXNumCells() &&
      y>=0 && y<grid_->getYNumCells() &&
      z>=0 && z<grid_->getZNumCells());
}

inline std::string OccupancyGrid::getReferenceFrame()
{
  return reference_frame_;
}

inline void OccupancyGrid::setReferenceFrame(const std::string &frame)
{
  reference_frame_ = frame;
}

inline void OccupancyGrid::addPointsToField(const std::vector<Eigen::Vector3d> &points)
{
  EigenSTL::vector_Vector3d pts(points.size());
  for(size_t i = 0; i < points.size(); ++i)
    pts[i] = Eigen::Vector3d(points[i].x(), points[i].y(), points[i].z());

  grid_->addPointsToField(pts);
}

}

#endif
