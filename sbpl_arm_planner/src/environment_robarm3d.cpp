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

#include <sbpl_arm_planner/environment_robarm3d.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

using namespace std;

namespace sbpl_arm_planner
{

EnvironmentROBARM3D::EnvironmentROBARM3D(OccupancyGrid *grid, RobotModel *rmodel, CollisionChecker *cc, ActionSet* as) : bfs_(NULL)
{
  grid_ = grid;
  rmodel_ = rmodel;
  cc_ = cc;
  as_ = as;
  EnvROBARM.Coord2StateIDHashTable = NULL;
  EnvROBARMCfg.bInitialized = false;
  near_goal = false;
  getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getXYZHeuristic;
}

EnvironmentROBARM3D::~EnvironmentROBARM3D()
{
  if(bfs_ != NULL)
    delete bfs_;

  for(size_t i = 0; i < EnvROBARM.StateID2CoordTable.size(); i++)
  {
    delete EnvROBARM.StateID2CoordTable.at(i);
    EnvROBARM.StateID2CoordTable.at(i) = NULL;
  }
  EnvROBARM.StateID2CoordTable.clear();

  if(EnvROBARM.Coord2StateIDHashTable != NULL)
  {
    delete [] EnvROBARM.Coord2StateIDHashTable;
    EnvROBARM.Coord2StateIDHashTable = NULL;
  }
}

bool EnvironmentROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
  MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;
  return true;
}

bool EnvironmentROBARM3D::InitializeEnv(const char* sEnvFile)
{
  ROS_INFO("[env] InitializeEnv is not implemented right now.");
  return true;
}

int EnvironmentROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return (*this.*getHeuristic_)(FromStateID,ToStateID);
}

int EnvironmentROBARM3D::GetGoalHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.goalHashEntry->stateID);
}

int EnvironmentROBARM3D::GetStartHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.startHashEntry->stateID);
}

int EnvironmentROBARM3D::SizeofCreatedEnv()
{
  return EnvROBARM.StateID2CoordTable.size();
}

void EnvironmentROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == EnvROBARM.goalHashEntry->stateID)
    bGoal = true;

  if(stateID == EnvROBARM.goalHashEntry->stateID && bVerbose)
  {
    //ROS_DEBUG_NAMED(fOut, "the state is a goal state\n");
    bGoal = true;
  }

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentROBARM3D::PrintEnv_Config(FILE* fOut)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  double dist=0;
  int endeff[3]={0};
  int path_length=0, nchecks=0;
  std::vector<int> scoord(num_joints_,0);
  std::vector<double> pose(6,0), angles(num_joints_,0), source_angles(num_joints_,0);

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();

  //goal state should be absorbing
  if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    return;

  //get X, Y, Z for the state
  EnvROBARM3DHashEntry_t* parent_entry = EnvROBARM.StateID2CoordTable[SourceStateID];

  //default coords of successor
  for(int i = 0; i < num_joints_; i++)
    scoord[i] = parent_entry->coord[i];

  //used for interpolated collision check
  coordToAngles(scoord, source_angles);

  ROS_DEBUG_NAMED(prms_.expands_log_, "\nstate %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  endeff: %3d %3d %3d",SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2]);
 

  int valid = 1;
  std::vector<Action> actions;
  if(!as_->getActionSet(source_angles, actions))
  {
    ROS_WARN("No successors found.");
    return;
  }

  // check actions for validity
  for (int i = 0; i < int(actions.size()); ++i)
  {
    valid = 1;
    for(size_t j = 0; j < actions[i].size(); ++j)
    {
      // check joint limits
      if(!rmodel_->checkJointLimits(actions[i][j]))
        valid = -1;

      //check for collisions
      if(!cc_->isStateValid(actions[i][j], prms_.verbose_, false, dist))
      {
        ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %0.3f is in collision.", i, dist);
        valid = -2;
      }

      if(valid < 1)
        break;
    }

    if(valid < 1)
      continue;

    // check for collisions along path from parent to first waypoint
    if(!cc_->isStateToStateValid(source_angles, angles, path_length, nchecks, dist))
    {
      ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
      valid = -3;
      break;
    }

    if(valid < 1)
      continue;

    // check for collisions between waypoints
    for(size_t j = 1; j < actions[i].size(); ++j)
    {
      if(!cc_->isStateToStateValid(actions[i][j-1], actions[i][j], path_length, nchecks, dist))
      {
        ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
        valid = -4;
        break;
      }
    }

    if(valid < 1)
      continue;

    // compute coords
    anglesToCoord(actions[i].back(), scoord);

    // get the successor
    EnvROBARM3DHashEntry_t* succ_entry;
    bool succ_is_goal_state = false;

    // get pose of planning link
    if(!rmodel_->computePlanningLinkFK(actions[i].back(), pose))
      continue;

    // discretize planning link pose
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);

    //check if this state meets the goal criteria
    if(isGoalState(pose, goal_))
    {
      succ_is_goal_state = true;

      for (int k = 0; k < num_joints_; k++)
        EnvROBARM.goalHashEntry->coord[k] = scoord[k];

      EnvROBARM.goalHashEntry->xyz[0] = endeff[0];
      EnvROBARM.goalHashEntry->xyz[1] = endeff[1];
      EnvROBARM.goalHashEntry->xyz[2] = endeff[2];
      EnvROBARM.goalHashEntry->state = actions[i].back();
      EnvROBARM.goalHashEntry->dist = dist;
    }

    //check if hash entry already exists, if not then create one
    if((succ_entry = getHashEntry(scoord, i, succ_is_goal_state)) == NULL)
    {
      succ_entry = createHashEntry(scoord, endeff);
      succ_entry->state = actions[i].back();
      succ_entry->dist = dist;

      ROS_DEBUG_NAMED(prms_.expands_log_, "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", succ_entry->stateID, i, int(succ_entry->dist), cost(parent_entry,succ_entry, succ_is_goal_state), GetFromToHeuristic(succ_entry->stateID, EnvROBARM.goalHashEntry->stateID), succ_entry->xyz[0],succ_entry->xyz[1],succ_entry->xyz[2]);
    }

    //put successor on successor list with the proper cost
    SuccIDV->push_back(succ_entry->stateID);
    CostV->push_back(cost(parent_entry, succ_entry, succ_is_goal_state));
  }

  expanded_states.push_back(SourceStateID);
}

void EnvironmentROBARM3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentROBARM3D::AreEquivalent(int StateID1, int StateID2)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: AreEquivalent is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in EnvROBARM..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::SetAllPreds(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

void EnvironmentROBARM3D::printHashTableHist()
{
  int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

  for(int  j = 0; j < EnvROBARM.HashTableSize; j++)
  {
    if((int)EnvROBARM.Coord2StateIDHashTable[j].size() == 0)
      s0++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 50)
      s1++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 100)
      s50++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 200)
      s100++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 300)
      s200++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 400)
      s300++;
    else
      slarge++;
  }
  ROS_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
      s0,s1, s50, s100, s200,s300,slarge);
}

EnvROBARM3DHashEntry_t* EnvironmentROBARM3D::getHashEntry(const std::vector<int> &coord, int action, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return EnvROBARM.goalHashEntry;

  int binid = getHashBin(coord);

#if DEBUG
  if ((int)EnvROBARM.Coord2StateIDHashTable[binid].size() > 500)
  {
    ROS_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", 
        binid, coord[0], int(EnvROBARM.Coord2StateIDHashTable[binid].size()));
    printHashTableHist();
  }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++)
  {
    int j = 0;

    for(j = 0; j < int(coord.size()); j++)
    {
      if(EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
        break;
    }

    if (j == int(coord.size()))
      return EnvROBARM.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvROBARM3DHashEntry_t* EnvironmentROBARM3D::createHashEntry(const std::vector<int> &coord, int endeff[3])
{
  int i;
  EnvROBARM3DHashEntry_t* HashEntry = new EnvROBARM3DHashEntry_t;

  HashEntry->coord = coord;

  memcpy(HashEntry->xyz, endeff, 3*sizeof(int));

  // assign a stateID to HashEntry to be used 
  HashEntry->stateID = EnvROBARM.StateID2CoordTable.size();

  //insert into the tables
  EnvROBARM.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = getHashBin(HashEntry->coord);

  //insert the entry into the bin
  EnvROBARM.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
  {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

void EnvironmentROBARM3D::initHeuristics()
{
  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  bfs_ = new BFS_3D(dimX, dimY, dimZ);
  ROS_DEBUG("[env] BFS is initialized.");
}

void EnvironmentROBARM3D::discretizeAngles()
{
  for(int i = 0; i < num_joints_; i++)
  {
    EnvROBARMCfg.coord_delta[i] = (2.0*M_PI) / prms_.angle_delta_;
    EnvROBARMCfg.coord_vals[i] = prms_.angle_delta_;
  }
}

int EnvironmentROBARM3D::cost(EnvROBARM3DHashEntry_t* HashEntry1, EnvROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal)
{
  if(prms_.use_uniform_cost_)
    return prms_.cost_multiplier_;
  else
  {
    // Max's suggestion is to just put a high cost on being close to
    // obstacles but don't provide some sort of gradient 
    if(int(HashEntry2->dist) < 7) // in cells
      return prms_.cost_multiplier_ * prms_.range1_cost_;
    else if(int(HashEntry2->dist) < 12)
      return prms_.cost_multiplier_ * prms_.range2_cost_;
    else if(int(HashEntry2->dist) < 17)
      return prms_.cost_multiplier_ * prms_.range3_cost_;
    else
      return prms_.cost_multiplier_;
  }
}

bool EnvironmentROBARM3D::initEnvConfig()
{
  int endeff[3] = {0};
  std::vector<int> coord(num_joints_,0);
  EnvROBARMCfg.coord_delta.resize(num_joints_);
  EnvROBARMCfg.coord_vals.resize(num_joints_);

  discretizeAngles();

  //initialize the map from Coord to StateID
  EnvROBARM.HashTableSize = 32*1024; //should be power of two
  EnvROBARM.Coord2StateIDHashTable = new std::vector<EnvROBARM3DHashEntry_t*>[EnvROBARM.HashTableSize];

  //initialize the map from StateID to Coord
  EnvROBARM.StateID2CoordTable.clear();

  //create an empty start state
  EnvROBARM.startHashEntry = createHashEntry(coord, endeff);

  //create the goal state
  EnvROBARM.goalHashEntry = createHashEntry(coord, endeff);

  return true;
}

bool EnvironmentROBARM3D::initEnvironment(std::string mprims_filename, std::string urdf, std::string srdf)
{
  //initialize the arm planner parameters
  prms_.initFromParamServer();

  /*
  FILE* mprims_fp=NULL;
  //parse motion primitives file
  if((mprims_fp=fopen(mprims_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open motion primitive file.");
    return false;
  }
  if(!prms_.initMotionPrims(mprims_fp))
  {
    SBPL_ERROR("Failed to parse motion primitive file.");
    fclose(mprims_fp);
    return false;
  }
  fclose(mprims_fp);
  */

  /*
  //initialize the environment & planning variables  
  if(!initGeneral())
  {
    SBPL_ERROR("Failed to initialize environment.");
    return false;
  }
  */

  //initialize Environment
  if(!initEnvConfig())
    return false;

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize BFS
  initHeuristics();
 
/* 
  if(prms_.verbose_)
  {
    prms_.printParams(std::string("sbpl_arm"));
    prms_.printMotionPrims(std::string("sbpl_arm"));
  }
*/

  //set heuristic function pointer
  getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getXYZHeuristic;

  //set 'Environment is Initialized' flag
  EnvROBARMCfg.bInitialized = true;

  ROS_INFO("[env] Environment has been initialized.");
  
  //for statistics purposes
  starttime = clock();
  return true;
}

bool EnvironmentROBARM3D::isGoalState(const std::vector<double> &pose, GoalState &goal)
{
  if(goal.type == XYZ_RPY_GOAL)
  {
    if(fabs(pose[0]-goal.pose[0]) <= goal.xyz_tolerance[0] && 
        fabs(pose[1]-goal.pose[1]) <= goal.xyz_tolerance[1] && 
        fabs(pose[2]-goal.pose[2]) <= goal.xyz_tolerance[2])
    {
      //log the amount of time required for the search to get close to the goal
      if(!near_goal)
      {
        time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
        near_goal = true;
        printf("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)\n", pose[0],pose[1],pose[2],goal.xyz_tolerance[0], goal.pose[0], goal.pose[1], goal.pose[2], time_to_goal_region,(int)expanded_states.size());
      }

      if(fabs(pose[3]-goal.pose[3]) <= goal.rpy_tolerance[0] && 
          fabs(pose[4]-goal.pose[4]) <= goal.rpy_tolerance[1] && 
          fabs(pose[5]-goal.pose[5]) <= goal.rpy_tolerance[2])
        return true;
    }
  }
  else if(goal.type == XYZ_GOAL)
  {
    if(fabs(pose[0]-goal.pose[0]) <= goal.xyz_tolerance[0] && 
        fabs(pose[1]-goal.pose[1]) <= goal.xyz_tolerance[1] && 
        fabs(pose[2]-goal.pose[2]) <= goal.xyz_tolerance[2])
      return true;
  }
  else
    ROS_ERROR("Unknown goal type.");

  return false;
}

int EnvironmentROBARM3D::getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist)
{
  int num_prims = 0, cost = 0;
  double diff = 0, max_diff = 0;

  if(from_config.size() != to_config.size())
    return -1;

/* NOTE: Not including forearm roll OR wrist roll movement to calculate mprim cost */

  for(size_t i = 0; i < 6; i++)
  {
    if(i == 4)
      continue;

    diff = fabs(angles::shortest_angular_distance(from_config[i], to_config[i]));
    if(max_diff < diff)
      max_diff = diff;
  }

  num_prims = max_diff / prms_.max_mprim_offset_ + 0.5;
  cost = num_prims * prms_.cost_multiplier_;

  ROS_DEBUG_NAMED("search", "from: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", angles::normalize_angle(from_config[0]),angles::normalize_angle(from_config[1]),angles::normalize_angle(from_config[2]),angles::normalize_angle(from_config[3]),angles::normalize_angle(from_config[4]),angles::normalize_angle(from_config[5]),angles::normalize_angle(from_config[6]));
  ROS_DEBUG_NAMED("search", "  to: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f diff: %0.2f num_prims: %d cost: %d (mprim_size: %0.3f)", to_config[0],to_config[1],to_config[2],to_config[3],to_config[4],to_config[5],to_config[6], max_diff, num_prims, cost, prms_.max_mprim_offset_);
  
  return cost;
}

int EnvironmentROBARM3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() 
      || ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif

  //get X, Y for the state
  EnvROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
  EnvROBARM3DHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

bool EnvironmentROBARM3D::setStartConfiguration(const std::vector<double> angles)
{
  double dist=100;
  int x,y,z;
  std::vector<double> pose(6,0);

  if(int(angles.size()) < num_joints_)
    return false;

  //get joint positions of starting configuration
  if(!rmodel_->computePlanningLinkFK(angles, pose))
    ROS_WARN("Unable to compute forward kinematics for initial robot state. Attempting to plan anyway.");

  //check joint limits of starting configuration but plan anyway
  if(!rmodel_->checkJointLimits(angles))
    ROS_WARN("Starting configuration violates the joint limits. Attempting to plan anyway.");

  //check if the start configuration is in collision but plan anyway
  if(!cc_->isStateValid(angles, prms_.verbose_, false, dist))
  {
    ROS_WARN("[env] The starting configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
  }

  //get arm position in environment
  anglesToCoord(angles, EnvROBARM.startHashEntry->coord);
  grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
  EnvROBARM.startHashEntry->xyz[0] = (int)x;
  EnvROBARM.startHashEntry->xyz[1] = (int)y;
  EnvROBARM.startHashEntry->xyz[2] = (int)z;
  return true;
}

bool EnvironmentROBARM3D::setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,is_6dof},{x2,y2,z2,r2,p2,y2,is_6dof}...}

  if(!EnvROBARMCfg.bInitialized)
  {
    SBPL_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    SBPL_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }

  goal_.pose.resize(6,0);
  goal_.pose[0] = goals[0][0];
  goal_.pose[1] = goals[0][1];
  goal_.pose[2] = goals[0][2];
  goal_.pose[3] = goals[0][3];
  goal_.pose[4] = goals[0][4];
  goal_.pose[5] = goals[0][5];
  goal_.xyz_tolerance[0] = tolerances[0][0];
  goal_.xyz_tolerance[1] = tolerances[0][1];
  goal_.xyz_tolerance[2] = tolerances[0][2];
  goal_.rpy_tolerance[0] = tolerances[0][3];
  goal_.rpy_tolerance[1] = tolerances[0][4];
  goal_.rpy_tolerance[2] = tolerances[0][5];
  goal_.type = goals[0][6];


  // set goal hash entry
  grid_->worldToGrid(goals[0][0], goals[0][1], goals[0][2], EnvROBARM.goalHashEntry->xyz[0],EnvROBARM.goalHashEntry->xyz[1], EnvROBARM.goalHashEntry->xyz[2]);

  for(int i = 0; i < num_joints_; i++)
    EnvROBARM.goalHashEntry->coord[i] = 0;

  ROS_DEBUG_NAMED("search", "time: %f", clock() / (double)CLOCKS_PER_SEC);
  ROS_DEBUG_NAMED("search", "A new goal has been set.");
  ROS_DEBUG_NAMED("search", "grid: %u %u %u (cells)  xyz: %.2f %.2f %.2f (meters)  (tol: %.3f) rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3f)", EnvROBARM.goalHashEntry->xyz[0],EnvROBARM.goalHashEntry->xyz[1], EnvROBARM.goalHashEntry->xyz[2], goal_.pose[0], goal_.pose[1], goal_.pose[2], goal_.xyz_tolerance[0], goal_.pose[3], goal_.pose[4], goal_.pose[5], goal_.rpy_tolerance[0]);


  // push obstacles into bfs grid
  ros::WallTime start = ros::WallTime::now();
  distance_field::PropagationDistanceField* df = grid_->getDistanceFieldPtr();
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);

  int walls=0;
  for (int z = 0; z < dimZ - 2; z++)
    for (int y = 0; y < dimY - 2; y++)
      for (int x = 0; x < dimX - 2; x++)
        if (df->getDistanceFromCell(x,y,z) <= heuristic_sphere_)
        {
          bfs_->setWall(x + 1, y + 1, z + 1);
          walls++;
        }
  
  double set_walls_time = (ros::WallTime::now() - start).toSec();
  ROS_DEBUG("[env] %0.5fsec to set walls in new bfs. (%d walls (%0.3f percent))", set_walls_time, walls, double(walls)/double(dimX*dimY*dimZ));
  start = ros::WallTime::now();
  bfs_->run(EnvROBARM.goalHashEntry->xyz[0], EnvROBARM.goalHashEntry->xyz[1], EnvROBARM.goalHashEntry->xyz[2]);
  ROS_DEBUG("[env] Time required to compute at least enough of the BFS to reach the start state: %0.4fsec", (ros::WallTime::now() - start).toSec());
  return true;
}

void EnvironmentROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);

  for (size_t i = 0; i < angles.size(); i++)
  {
    if(angles[i] >= M_PI)
      angles[i] = -2.0*M_PI + angles[i];
  }
}

void EnvironmentROBARM3D::printJointArray(FILE* fOut, EnvROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose)
{
  std::vector<double> angles(num_joints_,0);

  if(bGoal)
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);

  if(bVerbose)
    fprintf(fOut, "angles: ");

  for(int i = 0; i < int(angles.size()); i++)
  {
    if(i > 0)
      fprintf(fOut, "%-.3f ", angles[i]-angles[i-1]);
    else
      fprintf(fOut, "%-.3f ", angles[i]);
  }
  fprintf(fOut, "\n");
}

void EnvironmentROBARM3D::getExpandedStates(std::vector<std::vector<double> >* ara_states)
{
  std::vector<double> angles(num_joints_,0);
  std::vector<double>state(7,0);

  for(size_t i = 0; i < expanded_states.size(); ++i)
  {
    StateID2Angles(expanded_states[i],angles);
    rmodel_->computePlanningLinkFK(angles,state);
    state[6] = EnvROBARM.StateID2CoordTable[expanded_states[i]]->heur;
    ara_states->push_back(state);
  }
}

void EnvironmentROBARM3D::computeCostPerCell()
{
  ROS_ERROR("Fill in function to computeCostPerCell()");

  /*
  int largest_action=0;
  double gridcell_size, eucl_dist, max_dist = 0;
  std::vector<double> pose(6,0), start_pose(6,0), angles(num_joints_,0), start_angles(num_joints_,0);

  gridcell_size = grid_->getResolution();

  //starting at zeroed angles, find end effector position after each action
  rmodel_->computePlanningLinkFK(start_angles, start_pose);

  //iterate through all possible actions and find the one with the minimum cost per cell
  for (int i = 0; i < prms_.num_mprims_; i++)
  {
    for(int j = 0; j < num_joints_; j++)
      angles[j] = DEG2RAD(prms_.mprims_[i][j]);

    //starting at zeroed angles, find end effector position after each action
    if(!rmodel_->computePlanningLinkFK(angles, pose))
    {
      ROS_WARN("[env] Failed to compute cost per cell because forward kinematics is returning an error.");
      return;
    }

    eucl_dist = sqrt((start_pose[0]-pose[0])*(start_pose[0]-pose[0]) +
        (start_pose[1]-pose[1])*(start_pose[1]-pose[1]) +
        (start_pose[2]-pose[2])*(start_pose[2]-pose[2]));

    if (eucl_dist > max_dist)
    {
      max_dist = eucl_dist;
      largest_action = i;
    }
  }

  prms_.setCellCost(int(prms_.cost_multiplier_ / (max_dist/gridcell_size)));

  prms_.cost_per_meter_ = int(prms_.cost_per_cell_ / gridcell_size);

  ROS_INFO("[env] max_dist_traveled_per_smp: %0.3fm  cost per cell: %d  cost per meter: %d  (type: jointspace)", max_dist, prms_.cost_per_cell_,prms_.cost_per_meter_);
  */
}

int EnvironmentROBARM3D::getBFSCostToGoal(int x, int y, int z) const
{
  return bfs_->getDistance(x,y,z) * prms_.cost_per_cell_;
}

int EnvironmentROBARM3D::getXYZHeuristic(int FromStateID, int ToStateID)
{
  EnvROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];

  //get distance heuristic
  if(prms_.use_bfs_heuristic_)
    FromHashEntry->heur = getBFSCostToGoal(FromHashEntry->xyz[0], FromHashEntry->xyz[1], FromHashEntry->xyz[2]);
  else
  {
    double x, y, z;
    grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
    FromHashEntry->heur = getEuclideanDistance(x, y, z, goal_.pose[0], goal_.pose[1], goal_.pose[2]) * prms_.cost_per_meter_;
  }
  return FromHashEntry->heur;
}

}

