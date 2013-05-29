/*
 * Copyright (c) 2009, Maxim Likhachev
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

/** \Author: Benjamin Cohen /bcohen@willowgarage.com **/

#ifndef _BFS3D_
#define _BFS3D_

#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <ros/console.h>
#include <distance_field/propagation_distance_field.h>

using namespace std;

#define SMALL_NUM  0.00000001  
#define INFINITE_COST 1000000000 
 
#define DEBUG_TIME 0 
#define DIRECTIONS3D 26 
#define GOAL_TOLERANCE 0 

namespace sbpl_arm_planner {

typedef struct
{
  unsigned int g;
  int iterationclosed;
  int x;
  int y;
  int z;
} State3D;


class BFS3D
{
  class Cell3D
  {
    public:
      int x;
      int y;
      int z;
  };

  class FIFO
  {
    public:
      FIFO(int length);
      ~FIFO();
      bool empty();
      void clear();
      void insert(int x, int y, int z);
      void remove(int* x, int* y, int* z);

    private:
      int head_;
      int tail_;
      int size_;
      Cell3D *q_;
  };

  public:

    /**  \brief constructor
     * @param dim_x dim_x in cells
     * @param dim_y dim_y of grid in cells
     * @param dim_z dim_z of grid in cells
     * @param radius radius in cells of the point robot
     * @param cost_per_cell cost of traversing one cell in the grid*/
    BFS3D(int dim_x, int dim_y, int dim_z, int radius, int cost_per_cell);

    /** \brief destructor */
    ~BFS3D();

    /** \brief initialize the occupancy grid (required if not using
     * a distance field to describe the environment). */
    void init();
    
    /** \brief set a single goal 
     * @param goal a 3x1 vector {x,y,z}
     * @return true if success, false if goal is invalid */
    bool setGoal(std::vector<int> goal);
    
    /** \brief set a list of goals
     * @param goal an  nx3 vector {x,y,z}
     * @return true if success, false if all goals are invalid */
    bool setGoals(std::vector<std::vector<int> > goals);

    /** \brief compute a breadth first search from every cell to the goal(s)
     * @return true if it succeed and false otherwise */
    bool runBFS();

    /** \brief get distance to the goal in cells from (x,y,z) */
    int getDist(int x, int y, int z);

    /** \brief for debugging - print the occupancy grid to the terminal (beware poor formatting) */
    void printGrid();

    /** \brief for debugging - print the cost to goal from each cell */
    void printCostToGoal();

    /** \brief compute the shortest path from the specified cell to the
     * nearest goal.
     * @param start a 3x1 vector containing the {x,y,z} start position 
     * @param path an nx3 vector containing the waypoints
     * @return true if it was a success and false otherwise */
    bool getShortestPath(std::vector<int> start, std::vector<std::vector<int> > &path);

    /** \brief configure the distance field object that will be used as the
     * occupancy grid.
     * @param enable enable or disable the distance field
     * @param df pointer to distance field to use
    */
    void configDistanceField(bool enable, const distance_field::PropagationDistanceField* df);

    /** \brief output configuration values for debugging */
    void printConfig(FILE* fOut);

    void setRadius(double r);

    int getRadiusCells() { return int(radius_); };

    /* for two arm planner - no roll,pitch allowed */
    void setZInflation(int cells_above, int cells_below);
    void useResearchGrid(bool use_xygrid) { use_research_grid_ = use_xygrid;};
    void initializeXYGrid();
    void printXYPlane(int z);
    void printInflatedXYPlane(int z);
    void inflateXYGrid();
    void inflateXYGrid(int z_min, int z_max);
    void computeDistancestoNonfreeAreas(std::vector<std::vector<unsigned char> > &Grid2D, int width_x, int height_y, unsigned char obsthresh, std::vector<std::vector<double> > &disttoObs_incells);
    bool setGoal(int x, int y, int z);
    bool getShortestPath(int x, int y, int z, std::vector<std::vector<int> > &path);

  private:

    int dimX_;
    int dimY_;
    int dimZ_;
  
    int radius_;
    double radius_m_;

    std::vector<std::vector<int> > goal_;

    int cost_1_move_;
    int cost_sqrt2_move_;
    int cost_sqrt3_move_;

    bool enable_df_;
    bool use_research_grid_;
    const distance_field::PropagationDistanceField* df_;

    unsigned char*** grid3D_;

    int dist_length_;
    std::vector<int> dist_;

    void reInitializeState3D(State3D* state);
    void initializeState3D(State3D* state, int x, int y, int z);
    void create3DStateSpace(State3D**** statespace3D);
    void delete3DStateSpace(State3D**** statespace3D);
    inline int xyzToIndex(int x, int y, int z);
    void search3DwithFifo();
    void search3DwithQueue(State3D*** statespace);
    bool isGoal(const std::vector<int> &state);
    bool isValidCell(const int x, const int y, const int z);

    FIFO *q_;

    /* for two arm planner - no roll,pitch allowed */
    short unsigned int z_cells_above_;
    short unsigned int z_cells_below_;
    std::vector<std::vector<std::vector<unsigned char> > > xy_grid_;
    std::vector<std::vector<std::vector<double> > > inflatedxy_grid_; //in cells

};

inline int BFS3D::xyzToIndex(int x, int y, int z)
{
  int ret = x + y*dimX_ + z*dimX_*dimY_;
  if(ret < dist_length_)
    return ret;
  else
  {
    ROS_WARN("[bfs3d] out of bounds (%d %d %d) (index: %d  size: %d)\n", x,y,z,ret,dist_length_);
    return 0;
  }
}

}

#endif


