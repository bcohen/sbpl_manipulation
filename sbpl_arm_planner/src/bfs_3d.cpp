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

/** \author Benjamin Cohen, Mike Philips, Maxim Likhachev */

#include <sbpl_arm_planner/bfs_3d.h>

namespace sbpl_arm_planner {

BFS3D::BFS3D(int dim_x, int dim_y, int dim_z, int radius, int cost_per_cell)
{
  int fifo_size = 0;

  if(dim_x < 0 || dim_y < 0 || dim_z < 0)
    ROS_ERROR("Dimensions must have positive values. Fix this.\n");

  grid3D_ = NULL;
  df_ = NULL;

  dimX_ = dim_x;
  dimY_ = dim_y;
  dimZ_ = dim_z;
  radius_ = radius;

  cost_1_move_ = cost_per_cell;
  cost_sqrt2_move_ = cost_per_cell*sqrt(2.0);
  cost_sqrt3_move_ = cost_per_cell*sqrt(3.0);

  enable_df_ = false; //configDistanceField() should be called to enable
  radius_m_ = 0.1;
  use_research_grid_ = false;
  z_cells_above_ = 0;
  z_cells_below_ = 0;

  fifo_size = 2*dimX_*dimY_ + 2*dimY_*dimZ_ + 2*dimX_*dimZ_;
  q_ = new FIFO(fifo_size);
  ROS_DEBUG("[BFS3d] Allocated a FIFO of size %d", fifo_size);      
  ROS_DEBUG("[bfs3d] grid dimensions: %d %d %d\n",dimX_, dimY_,dimZ_);
}

BFS3D::~BFS3D()
{
  if (grid3D_ != NULL)
  {
    for (int x = 0; x < dimX_; x++)
    {
      for (int y = 0; y < dimY_; y++)
        delete [] grid3D_[x][y];
      delete [] grid3D_[x];
    }
    delete [] grid3D_;
    grid3D_ = NULL;
  }

  if(q_ != NULL)
    delete q_;
}

void BFS3D::init()
{
  int x,y,z;
  grid3D_ = new unsigned char** [dimX_];
  for (x = 0; x < dimX_; x++)
  {
    grid3D_[x] = new unsigned char* [dimY_];
    for (y = 0; y < dimY_; y++)
    {
      grid3D_[x][y] = new unsigned char [dimZ_];
      for (z = 0; z < dimZ_; z++)
      {
        grid3D_[x][y][z] = 255;
      }
    }
  }
}

bool BFS3D::setGoal(std::vector<int> goal)
{
  if(goal.empty() || goal.size() < 3)
    return false;

  goal_.clear();

  if(goal[0] < dimX_ && goal[1] < dimY_ && goal[2] < dimZ_)
    goal_.push_back(goal);

  if(goal_.empty())
  { 
    ROS_ERROR("[bfs3d] Error: No valid goals were received.");
    return false;
  }
  return true;
}

bool BFS3D::setGoal(int x, int y, int z)
{
  std::vector<int> goal(3);
  goal[0] = x;
  goal[1] = y;
  goal[2] = z;

  return setGoal(goal);
}

bool BFS3D::setGoals(std::vector<std::vector<int> > goals)
{
  if(goals.size() <= 0)
  {
    ROS_DEBUG("[bfs3d] No goal cell received. Exiting.");
    return false;
  }

  goal_.clear();

  //check if received goals are valid
  for(unsigned int i = 0; i < goals.size(); ++i)
  {
    if(goals[i].size() < 3)
      continue;

    if(goals[i][0] < dimX_ && goals[i][1] < dimY_ && goals[i][2] < dimZ_)
      goal_.push_back(goals[i]);
    else
      ROS_DEBUG("Goal: %u %u %u is invalid.",goals[i][0],goals[i][1],goals[i][2]);
  }

  if(goal_.empty())
  {
    ROS_DEBUG("Error: No valid goals were received.\n");
    return false;
  }
  return true;
}

void BFS3D::reInitializeState3D(State3D* state)
{
  state->g = INFINITE_COST;
  state->iterationclosed = 0;
}

void BFS3D::initializeState3D(State3D* state, int x, int y, int z)
{
  state->g = INFINITE_COST;
  state->iterationclosed = 0;
  state->x = x;
  state->y = y;
  state->z = z;
}

void BFS3D::create3DStateSpace(State3D**** statespace3D)
{
  int  x,y,z;

  *statespace3D = new State3D** [dimX_];
  for (x = 0; x < dimX_; x++)
  {
    (*statespace3D)[x] = new State3D* [dimY_];
    for(y = 0; y < dimY_; y++)
    {
      (*statespace3D)[x][y] = new State3D [dimZ_];
      for(z = 0; z < dimZ_; z++)
      {
        initializeState3D(&(*statespace3D)[x][y][z],x,y,z);
      }
    }
  }
}

void BFS3D::delete3DStateSpace(State3D**** statespace3D)
{
  int x,y;

  if((*statespace3D) != NULL)
  {
    for (x = 0; x < dimX_; x++)
    {
      for (y = 0; y < dimY_; y++)
        delete [] (*statespace3D)[x][y];

      delete [] (*statespace3D)[x];
    }
    delete [] (*statespace3D);
    (*statespace3D) = NULL;
  }
}

bool BFS3D::runBFS()
{
#if DEBUG_TIME
  clock_t currenttime = clock();
#endif

  if(goal_.empty())
  {
    ROS_ERROR("[bfs3d] Goal location is not set. Exiting.\n");
    return false;
  }

  dist_length_ =  (dimX_-1) + (dimY_-1)*(dimX_) + (dimZ_-1)*(dimX_)*(dimY_) + 1;
  dist_.resize(dist_length_);

  search3DwithFifo();
  
/* Uncomment this if using a queue:
 
  State3D*** statespace3D;
  create3DStateSpace(&statespace3D);
  search3DwithQueue(statespace3D);
  delete3DStateSpace(&statespace3D);
*/

#if DEBUG_TIME
  ROS_DEBUG("completed in %.3f seconds.\n", double(clock()-currenttime) / CLOCKS_PER_SEC);
#endif

  return true;
}

int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

/* Mike Philips */
void BFS3D::search3DwithFifo()
{
  for(int x=0; x<dimX_; x++){
    for(int y=0; y<dimY_; y++){
      for(int z=0; z<dimZ_; z++){
        //statespace[x][y][z].g = INFINITE_COST;
        dist_[xyzToIndex(x,y,z)] = INFINITE_COST;
      }
    }
  }

  q_->clear();
  for(unsigned int i=0; i<goal_.size(); i++){
    int x = goal_[i][0];
    int y = goal_[i][1];
    int z = goal_[i][2];
    //statespace[x][y][z].g = 0;
    dist_[xyzToIndex(x,y,z)] = 0;
    q_->insert(x,y,z);
  }
}

int BFS3D::getDist(int x, int y, int z)
{
  int idx = xyzToIndex(x,y,z);
  int d = dist_[idx];
  while(d==INFINITE_COST && !q_->empty()){
    int x,y,z;
    q_->remove(&x,&y,&z);

    bool onBoundary = x==0 || x==dimX_-1 || y==0 || y==dimY_-1 || z==0 || z==dimZ_-1;

    int parentDist = dist_[xyzToIndex(x,y,z)];
    for(int i=0; i<DIRECTIONS3D; i++){
      int newX = x+dx[i];
      int newY = y+dy[i];
      int newZ = z+dz[i];
      if((!onBoundary || (newX>=0 && newX<dimX_ && newY>=0 && newY<dimY_ && newZ>=0 && newZ<dimZ_)) && //check if the successor is in work space
          isValidCell(newX,newY,newZ) && //is not an obstacle
          (dist_[xyzToIndex(newX,newY,newZ)] == INFINITE_COST)){ //and has not already been put in the queue
        q_->insert(newX,newY,newZ);
        dist_[xyzToIndex(newX,newY,newZ)] = parentDist + cost_1_move_;
      }
    }
    
    d = dist_[idx];
  }
  return d;
}


BFS3D::FIFO::FIFO(int length) : size_(length)
{
  head_ = 0;
  tail_ = 0;

  q_ = new Cell3D[size_];
}

BFS3D::FIFO::~FIFO()
{
  if(q_ != NULL)
  {
    delete [] q_;
    q_ = NULL;
  }
}

void BFS3D::FIFO::clear()
{
  head_ = 0;
  tail_ = 0;
}

bool BFS3D::FIFO::empty()
{
  return head_ == tail_;
}

void BFS3D::FIFO::insert(int x, int y, int z)
{
  q_[head_].x = x;
  q_[head_].y = y;
  q_[head_].z = z;
  if(head_==tail_-1 || (tail_==0 && head_==size_-1)){
    ROS_ERROR("FIFO FULL! This shouldn't have happened. Are you setting the correct size for the FIFO in the constructor? Exiting. ");
    exit(0);
  }
  head_++;
  if(head_ == size_)
    head_ = 0;
}

void BFS3D::FIFO::remove(int* x, int* y, int* z)
{
  *x = q_[tail_].x;
  *y = q_[tail_].y;
  *z = q_[tail_].z;
  tail_++;
  if(tail_ == size_)
    tail_ = 0;
}

void BFS3D::search3DwithQueue(State3D*** statespace)
{
  State3D* ExpState;
  int newx, newy, newz;
  int x,y,z;
  unsigned int g_temp;

  //these are added here temporarily. should be in the class
  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

  //create a queue
  queue<State3D*> Queue;

  //initialize to infinity all
  for (x = 0; x < dimX_; x++)
  {
    for (y = 0; y < dimY_; y++)
    {
      for (z = 0; z < dimZ_; z++)
      {
        dist_[xyzToIndex(x,y,z)] = INFINITE_COST;
        reInitializeState3D(&statespace[x][y][z]);
      }
    }
  }
  
  //initialization - throw starting states on queue with g cost = 0
  for(unsigned int i = 0; i < goal_.size(); ++i)
  {
    statespace[goal_[i][0]][goal_[i][1]][goal_[i][2]].g = 0;
    Queue.push(&statespace[goal_[i][0]][goal_[i][1]][goal_[i][2]]);
  }

  //expand all of the states
  while((int)Queue.size() > 0)
  {
    //get the state to expand
    ExpState = Queue.front();

    Queue.pop();

    //it may be that the state is already closed
    if(ExpState->iterationclosed == 1)
      continue;

    //close it
    ExpState->iterationclosed = 1;

    //set the corresponding distances to the goal
    dist_[xyzToIndex(ExpState->x, ExpState->y, ExpState->z)] = ExpState->g;

    //iterate through neighbors
    for(int d = 0; d < DIRECTIONS3D; d++)
    {
      newx = ExpState->x + dx[d];
      newy = ExpState->y + dy[d];
      newz = ExpState->z + dz[d];

      //make sure it is inside the map and has no obstacle
      if(0 > newx || newx >= dimX_ || 0 > newy || newy >= dimY_ || 0 > newz || newz >= dimZ_)
        continue;

      if(!isValidCell(newx,newy,newz))
        continue;
 
      if(statespace[newx][newy][newz].iterationclosed == 0)
      {
       //insert into the stack
        Queue.push(&statespace[newx][newy][newz]);

        //set the g-value
        if (ExpState->x != newx && ExpState->y != newy && ExpState->z != newz)
          g_temp = ExpState->g + cost_sqrt3_move_;
        else if ((ExpState->y != newy && ExpState->z != newz) ||
            (ExpState->x != newx && ExpState->z != newz) ||
            (ExpState->x != newx && ExpState->y != newy))
          g_temp = ExpState->g + cost_sqrt2_move_;
        else
          g_temp = ExpState->g + cost_1_move_;

        if(statespace[newx][newy][newz].g > g_temp)
          statespace[newx][newy][newz].g = g_temp;
      }
    }
  }
}

bool BFS3D::isGoal(const std::vector<int> &state)
{
  for(unsigned int i = 0; i < goal_.size(); ++i)
  {
    if((state[0] <= goal_[i][0]+GOAL_TOLERANCE && state[0] >= goal_[i][0]-GOAL_TOLERANCE) && (state[1] <= goal_[i][1]+GOAL_TOLERANCE && state[1] >= goal_[i][1]-GOAL_TOLERANCE) && (state[2] <= goal_[i][2]+GOAL_TOLERANCE && state[2] >= goal_[i][2]-GOAL_TOLERANCE))
    {
      //ROS_PRINTF("the goal is found: %d %d %d\n", state[0],state[1],state[2]);
      return true;
    }
  }
  return false;
}

bool BFS3D::getShortestPath(std::vector<int> start, std::vector<std::vector<int> > &path)
{
  int val = 0, cntr = 0, min_val = INFINITE_COST;
  std::vector<int> state(3,0);
  std::vector<int> next_state(3,0);
  int newx,newy,newz;

  //make sure the while loop eventually stops
  int max_path_length = dimX_*dimY_;

  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

  path.resize(0);
  next_state[0] = start[0];
  next_state[1] = start[1];
  next_state[2] = start[2];

  while(!isGoal(next_state) || cntr > max_path_length)
  {
    state = next_state;
    min_val = INFINITE_COST;

    //iterate through neighbors
    for(int d = 0; d < DIRECTIONS3D; d++)
    {
      newx = state[0] + dx[d];
      newy = state[1] + dy[d];
      newz = state[2] + dz[d];

      //make sure it is inside the map and has no obstacle
      if(0 > newx || newx >= dimX_ || 0 > newy || newy >= dimY_ || 0 > newz || newz >= dimZ_)
        continue;

      val = dist_[xyzToIndex(newx,newy,newz)];

      if(val >= INFINITE_COST)
        continue;

      if (state[0] != newx && state[1] != newy && state[2] != newz)
        val += cost_sqrt3_move_;
      else if ((state[1] != newy && state[2] != newz) ||
          (state[0] != newx && state[2] != newz) ||
          (state[0] != newx && state[1] != newy))
        val += cost_sqrt2_move_;
      else
        val += cost_1_move_;

      if(val < min_val)
      {
        min_val = val;
        next_state[0] = newx;
        next_state[1] = newy;
        next_state[2] = newz;
      }
    }
    path.push_back(next_state);

    cntr++;
  }

  //unable to find path
  if(cntr > max_path_length)
  {
    ROS_WARN("[bfs3d] Unable to find path to goal. Exiting.");
    path.clear();
    return false;
  }

  return true;
}

bool BFS3D::getShortestPath(int x, int y, int z, std::vector<std::vector<int> > &path)
{
  int val = 0, cntr = 0, min_val = INFINITE_COST;
  std::vector<int> state(3,0);
  std::vector<int> next_state(4,0);
  int newx,newy,newz;

  //make sure the while loop eventually stops
  int max_path_length = dimX_*dimY_;

  path.clear();
  next_state[0] = x;
  next_state[1] = y;
  next_state[2] = z;
  next_state[3] = getDist(x,y,z);

  //add start state to the path
  path.push_back(next_state);

  while(!isGoal(next_state) || cntr > max_path_length)
  {
    state = next_state;
    min_val = INFINITE_COST;

    //iterate through neighbors
    for(int d = 0; d < DIRECTIONS3D; d++)
    {
      newx = state[0] + dx[d];
      newy = state[1] + dy[d];
      newz = state[2] + dz[d];

      val = getDist(newx,newy,newz);

      if(val < min_val)
      {
        min_val = val;
        next_state[0] = newx;
        next_state[1] = newy;
        next_state[2] = newz;
        next_state[3] = min_val;   // added 8/30/11 for debug output
      }
    }
    ROS_DEBUG("cntr: %d   xyz: %d %d %d val: %d", cntr, next_state[0],next_state[1],next_state[2],min_val);
    path.push_back(next_state);

    cntr++;
  }

  //unable to find path
  if(cntr > max_path_length)
  {
    ROS_WARN("[bfs3d] Unable to find path to goal. Exiting.");
    path.clear();
    return false;
  }

  return true;
}

void BFS3D::configDistanceField(bool enable, const distance_field::PropagationDistanceField* df)
{
  enable_df_ = enable;
  df_ = df;

  radius_m_ = double(radius_ * df_->getResolution(distance_field::PropagationDistanceField::DIM_X));
}


bool BFS3D::isValidCell(const int x, const int y, const int z)
{
  if(enable_df_)
  {
    if(use_research_grid_)
    {
      //out of bounds
      if(z+z_cells_above_ >= dimZ_)
        return false;

      for(int i = z-z_cells_below_; i <= z+z_cells_above_; ++i)
        if(inflatedxy_grid_[i][y][x]*df_->getResolution(distance_field::PropagationDistanceField::DIM_X) <= radius_m_)
          return false;

      /*
      if(inflatedxy_grid_[z][y][x]*df_->getResolution(distance_field::PropagationDistanceField::DIM_X) <= radius_m_)
        return false;
      */
    }
    else
      if(df_->getDistanceFromCell(x,y,z) <= radius_m_)
        return false;
  }
  else
  {  
    if(grid3D_[x][y][z] <= radius_)
      return false;
  }
  return true;
}

void BFS3D::printConfig(FILE* fOut)
{
  fprintf(fOut,"BFS3D Configuration:\n");
  fprintf(fOut,"dimX: %d   dimY: %d   dimZ: %d\n",dimX_,dimY_,dimZ_);
  fprintf(fOut,"robot radius(cells): %d   robot radius(meters): %0.3f\n",radius_,radius_m_);
}

void BFS3D::printGrid()
{
  for(int z = 0; z < dimZ_; ++z)
  {
    ROS_DEBUG("---------------------------------");
    ROS_DEBUG("z: %d",z);
    for(int x = 0; x < dimX_; ++x)
    {
      for(int y = 0; y < dimY_; ++y)
        ROS_DEBUG("%d ", grid3D_[x][y][z]);
    }
  }
}

void BFS3D::printCostToGoal()
{
  {
    for(int z = 0; z < dimZ_; ++z)
    {
      ROS_DEBUG("---------------------------------");
      ROS_DEBUG("z: %d",z);
      for(int x = 0; x < dimX_; ++x)
      {
        for(int y = 0; y < dimY_; ++y)
          ROS_DEBUG("%d ", dist_[xyzToIndex(x,y,z)]);
      }
    }
  }
}

void BFS3D::setRadius(double r)
{
  radius_m_ = r;
  radius_ = r / df_->getResolution(distance_field::PropagationDistanceField::DIM_X) + 0.5;
}

void BFS3D::setZInflation(int cells_above, int cells_below)
{
  z_cells_above_ = cells_above;
  z_cells_below_ = cells_below;
  ROS_INFO("[bfs3d] Z Inflation set to %d cells above and %d cells below.",cells_above, cells_below);
}

void BFS3D::initializeXYGrid()
{
  short unsigned int x,y,z;
  xy_grid_.resize(dimZ_,std::vector<std::vector<unsigned char> >(dimY_, std::vector<unsigned char>(dimX_,0)));
  for (z = 0; z < dimZ_; z++)
  {
    for (y = 0; y < dimY_; y++)
    {
      for (x = 0; x < dimX_; x++)
      {
        if(df_->getDistanceFromCell(x,y,z) <= 0)
          xy_grid_[z][y][x] = 100;
      }
    }
  }
  inflatedxy_grid_.resize(dimZ_,std::vector<std::vector<double> >(dimY_, std::vector<double>(dimX_,0)));
}

void BFS3D::inflateXYGrid()
{
  inflateXYGrid(0,dimZ_-1);
}

void BFS3D::inflateXYGrid(int z_min, int z_max)
{
  ROS_INFO("[bfs3d] About to inflate the XY grid.");
  for(int z = z_min; z <= z_max; ++z)
  {
    computeDistancestoNonfreeAreas(xy_grid_[z], dimY_, dimX_,1, inflatedxy_grid_[z]);
  }
  ROS_INFO("[bfs3d] Completed the inflation of the XY grid.");
}

void BFS3D::printXYPlane(int z)
{
  ROS_INFO("[bfs3d] Dimensions of XY grid: DimZ: %d  DimY: %d DimX: %d)\n",int(xy_grid_.size()),int(xy_grid_[0].size()),int(xy_grid_[0][0].size()));
  printf("XY Plane with z=%d\n",z);
  for(int x = 0; x < dimX_; ++x)
  {
    for(int y = 0; y < dimY_; ++y)
    {
      printf("%d ",int(xy_grid_[z][y][x]));
    }
    printf("\n");
  }
}

void BFS3D::printInflatedXYPlane(int z)
{
  ROS_INFO("[bfs3d] Dimensions of InflatedXYGrid: DimZ: %d  DimY: %d DimX: %d\n",int(inflatedxy_grid_.size()),int(inflatedxy_grid_[0].size()),int(inflatedxy_grid_[0][0].size()));
  printf("Inflated XY Plane with z=%d\n",z);
  for(int x = 0; x < dimX_; ++x)
  {
    for(int y = 0; y < dimY_; ++y)
    {
      printf("%0.1f ",inflatedxy_grid_[z][y][x]);
    }
    printf("\n");
  }
}

//computes 8-connected distances to obstacles and non-free areas in two linear passes and returns them in disttoObs_incells 
//and disttoNonfree_incells arrays. The distances are in terms of the number of cells but are floats. These distances
//can then be converted into the actual distances using the actual discretization values
//areas outside of the map are considered to be obstacles
void BFS3D::computeDistancestoNonfreeAreas(std::vector<std::vector<unsigned char> > &Grid2D, int width_x, int height_y, unsigned char obsthresh, std::vector<std::vector<double> > &disttoObs_incells)
{
  int x,y,nbrx,nbry;
  double mindisttoObs;
  double maxDist = (double)(min(width_x, height_y));
  double disttoObs;
  int dir;
  const int NUMOF2DQUASIDIRS = 4;

  // for quasi-Euclidean distance transform
  // going left-to-right, top-to-bottom
  int dxdownlefttoright_[NUMOF2DQUASIDIRS];
  int dydownlefttoright_[NUMOF2DQUASIDIRS];
  int dxdownrighttoleft_[NUMOF2DQUASIDIRS];
  int dydownrighttoleft_[NUMOF2DQUASIDIRS];

  // going right-to-left, bottom-to-top
  int dxuprighttoleft_[NUMOF2DQUASIDIRS];
  int dyuprighttoleft_[NUMOF2DQUASIDIRS];
  int dxuplefttoright_[NUMOF2DQUASIDIRS];
  int dyuplefttoright_[NUMOF2DQUASIDIRS];

  // distances to the above nbrs
  double distdownlefttoright_[NUMOF2DQUASIDIRS];
  double distdownrighttoleft_[NUMOF2DQUASIDIRS];
  double distuprighttoleft_[NUMOF2DQUASIDIRS];
  double distuplefttoright_[NUMOF2DQUASIDIRS];

  // and for distance transform:
  // increasing x (outer)
  // increasing y (inner)
  //  [2]
  //  [1][s]
  //  [0][3]
  dxdownlefttoright_[0] = -1; dydownlefttoright_[0] = -1;
  dxdownlefttoright_[1] = -1; dydownlefttoright_[1] = 0;
  dxdownlefttoright_[2] = -1; dydownlefttoright_[2] = 1;
  dxdownlefttoright_[3] = 0; dydownlefttoright_[3] = -1;

  // increasing x (outer)
  // decreasing y (inner)
  //  [2][3]
  //  [1][s]
  //  [0] 
  dxdownrighttoleft_[0] = -1; dydownrighttoleft_[0] = -1;
  dxdownrighttoleft_[1] = -1; dydownrighttoleft_[1] = 0;
  dxdownrighttoleft_[2] = -1; dydownrighttoleft_[2] = 1;
  dxdownrighttoleft_[3] = 0; dydownrighttoleft_[3] = 1;

  // decreasing x (outer)
  // decreasing y (inner)
  //  [3][2]
  //  [s][1]
  //     [0] 
  dxuprighttoleft_[0] = 1; dyuprighttoleft_[0] = -1;
  dxuprighttoleft_[1] = 1; dyuprighttoleft_[1] = 0;
  dxuprighttoleft_[2] = 1; dyuprighttoleft_[2] = 1;
  dxuprighttoleft_[3] = 0; dyuprighttoleft_[3] = 1;

  // decreasing x (outer)
  // increasing y (inner)
  //     [2]
  //  [s][1]
  //  [3][0] 
  dxuplefttoright_[0] = 1; dyuplefttoright_[0] = -1;
  dxuplefttoright_[1] = 1; dyuplefttoright_[1] = 0;
  dxuplefttoright_[2] = 1; dyuplefttoright_[2] = 1;
  dxuplefttoright_[3] = 0; dyuplefttoright_[3] = -1;

  // insert the corresponding distances
  distdownlefttoright_[0] = (double)1.414;
  distdownlefttoright_[1] = (double)1.0;
  distdownlefttoright_[2] = (double)1.414;
  distdownlefttoright_[3] = (double)1.0;

  distdownrighttoleft_[0] = (double)1.414;
  distdownrighttoleft_[1] = (double)1.0;
  distdownrighttoleft_[2] = (double)1.414;
  distdownrighttoleft_[3] = (double)1.0;

  distuprighttoleft_[0] = (double)1.414;
  distuprighttoleft_[1] = (double)1.0;
  distuprighttoleft_[2] = (double)1.414;
  distuprighttoleft_[3] = (double)1.0;

  distuplefttoright_[0] = (double)1.414;
  distuplefttoright_[1] = (double)1.0;
  distuplefttoright_[2] = (double)1.414;
  distuplefttoright_[3] = (double)1.0;

  // step through the map from top to bottom,
  // alternating left-to-right then right-to-left
  // This order maintains the invariant that the min distance for each
  // cell to all previously-visited obstacles is accurate
  for(x = 0; x < width_x; x++)
  {
    // move from left to right
    if (x%2 == 0) {

      for(y = 0; y < height_y; y++)
      {
        mindisttoObs = maxDist; // initialize to max distance

        // if cell is an obstacle, set disttoObs to 0 and continue
        if (Grid2D[x][y] >= obsthresh){
          disttoObs_incells[x][y] = 0;
          continue;
        }

        //iterate over predecessors
        for(dir = 0; dir < NUMOF2DQUASIDIRS; dir++){
          nbrx = x + dxdownlefttoright_[dir];
          nbry = y + dydownlefttoright_[dir];

          //make sure it is inside the map and has no obstacle
          // compute min cost to an obstacle for each cell, using 
          // *just* the cells already computed this pass for checking distance
          if(nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y){
            disttoObs = distdownlefttoright_[dir];
          }
          else
          {
            disttoObs = distdownlefttoright_[dir] + disttoObs_incells[nbrx][nbry];
          }

          if (disttoObs < mindisttoObs)
            mindisttoObs = disttoObs;
        }//over preds

        disttoObs_incells[x][y] = mindisttoObs;
      }

    } else {

      // move from right to left
      for(y = height_y-1; y >= 0; y--)
      {

        mindisttoObs = maxDist; // initialize to max distance

        // if cell is an obstacle, set disttoObs to 0 and continue
        if (Grid2D[x][y] >= obsthresh){
          disttoObs_incells[x][y] = 0;
          continue;
        }

        //iterate over predecessors
        for(dir = 0; dir < NUMOF2DQUASIDIRS; dir++)
        {
          nbrx = x + dxdownrighttoleft_[dir];
          nbry = y + dydownrighttoleft_[dir];

          //make sure it is inside the map and has no obstacle
          // compute min cost to an obstacle for each cell, using 
          // *just* the cells already computed this pass for checking distance
          if(nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y){
            disttoObs = distdownrighttoleft_[dir];
          } else {
            disttoObs = distdownrighttoleft_[dir] + disttoObs_incells[nbrx][nbry];
          }

          if (disttoObs < mindisttoObs)
            mindisttoObs = disttoObs;
        }

        disttoObs_incells[x][y] = mindisttoObs;
      }
      //ROS_PRINTF("x=%d\n", x);
    }
  }

  // step through the map from bottom to top
  for(x = width_x-1; x >= 0; x--)
  {

    // move from right to left
    if (x%2 == 0) {

      for(y = height_y-1; y >= 0; y--)
      {

        // initialize to current distance
        mindisttoObs = disttoObs_incells[x][y];

        //iterate over predecessors
        for(dir = 0; dir < NUMOF2DQUASIDIRS; dir++)
        {
          nbrx = x + dxuprighttoleft_[dir];
          nbry = y + dyuprighttoleft_[dir];

          //make sure it is inside the map and has no obstacle
          // compute min cost to an obstacle for each cell, using 
          // *just* the cells already computed this pass for checking distance
          if(nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y){
            disttoObs = distuprighttoleft_[dir];
          } else {
            disttoObs = distuprighttoleft_[dir] + disttoObs_incells[nbrx][nbry];
          }

          if (disttoObs < mindisttoObs)
            mindisttoObs = disttoObs;
        }//over preds

        disttoObs_incells[x][y] = mindisttoObs;
      }//for y        
    } else {

      // move from left to right
      for(y = 0; y< height_y; y++)
      {
        // initialize to current distance
        mindisttoObs = disttoObs_incells[x][y];

        //iterate over predecessors
        for(dir = 0; dir < NUMOF2DQUASIDIRS; dir++)
        {
          nbrx = x + dxuplefttoright_[dir];
          nbry = y + dyuplefttoright_[dir];

          //make sure it is inside the map and has no obstacle
          // compute min cost to an obstacle for each cell, using 
          // *just* the cells already computed this pass for checking distance
          if(nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y){
            disttoObs = distuplefttoright_[dir];
          } else {
            disttoObs = distuplefttoright_[dir] + disttoObs_incells[nbrx][nbry];
          }

          if (disttoObs < mindisttoObs)
            mindisttoObs = disttoObs;
        }

        disttoObs_incells[x][y] = mindisttoObs;
      }//over y                
    }//direction
  }//over x
}

}
