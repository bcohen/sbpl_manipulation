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
//#include <bfs3d/BFS_Util.hpp>
#include <leatherman/viz.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

using namespace std;

namespace sbpl_arm_planner
{

EnvironmentROBARM3D::EnvironmentROBARM3D(OccupancyGrid *grid, RobotModel *rmodel, CollisionChecker *cc, ActionSet* as, PlanningParams *pm) : bfs_(NULL)
{
  grid_ = grid;
  rmodel_ = rmodel;
  cc_ = cc;
  as_ = as;
  prm_ = pm;
  getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getXYZHeuristic;
}

EnvironmentROBARM3D::~EnvironmentROBARM3D()
{
  if(bfs_ != NULL)
    delete bfs_;

  for(size_t i = 0; i < pdata_.StateID2CoordTable.size(); i++)
  {
    delete pdata_.StateID2CoordTable.at(i);
    pdata_.StateID2CoordTable.at(i) = NULL;
  }
  pdata_.StateID2CoordTable.clear();

  if(pdata_.Coord2StateIDHashTable != NULL)
  {
    delete [] pdata_.Coord2StateIDHashTable;
    pdata_.Coord2StateIDHashTable = NULL;
  }
}

bool EnvironmentROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  MDPCfg->goalstateid = pdata_.goal_entry->stateID;
  MDPCfg->startstateid = pdata_.start_entry->stateID;
  return true;
}

int EnvironmentROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return (*this.*getHeuristic_)(FromStateID,ToStateID);
}

int EnvironmentROBARM3D::GetGoalHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, pdata_.goal_entry->stateID);
}

int EnvironmentROBARM3D::GetStartHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, pdata_.start_entry->stateID);
}

int EnvironmentROBARM3D::SizeofCreatedEnv()
{
  return pdata_.StateID2CoordTable.size();
}

void EnvironmentROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvROBARM3DHashEntry_t* HashEntry = pdata_.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == pdata_.goal_entry->stateID)
    bGoal = true;

  if(stateID == pdata_.goal_entry->stateID && bVerbose)
  {
    //ROS_DEBUG_NAMED(fOut, "the state is a goal state\n");
    bGoal = true;
  }

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentROBARM3D::PrintEnv_Config(FILE* fOut)
{
  ROS_ERROR("ERROR in pdata_... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  double dist=0;
  int endeff[3]={0};
  int path_length=0, nchecks=0;
  std::vector<int> scoord(prm_->num_joints_,0);
  std::vector<double> pose(6,0), angles(prm_->num_joints_,0), source_angles(prm_->num_joints_,0);

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();

  //goal state should be absorbing
  if(SourceStateID == pdata_.goal_entry->stateID)
    return;

  //get X, Y, Z for the state
  EnvROBARM3DHashEntry_t* parent_entry = pdata_.StateID2CoordTable[SourceStateID];

  if(int(parent_entry->coord.size()) < prm_->num_joints_)
    ROS_ERROR("Parent hash entry has broken coords. (# coords: %d)", int(parent_entry->coord.size()));
  //default coords of successor
  for(int i = 0; i < prm_->num_joints_; i++)
    scoord[i] = parent_entry->coord.at(i);

  //used for interpolated collision check
  coordToAngles(scoord, source_angles);

  ROS_DEBUG_NAMED(prm_->expands_log_, "\nstate %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  endeff: %3d %3d %3d",SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2]);
 

  int valid = 1;
  std::vector<Action> actions;
  if(!as_->getActionSet(source_angles, actions))
  {
    ROS_WARN("Failed to get successors.");
    return;
  }

  ROS_DEBUG_NAMED(prm_->expands_log_, "[parent: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  xyz: %3d %3d %3d  #_actions: %d  heur: %d dist: %0.3f", SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2], int(actions.size()), getXYZHeuristic(SourceStateID, 1), double(bfs_->getDistance(parent_entry->xyz[0],parent_entry->xyz[1], parent_entry->xyz[2])) * grid_->getResolution());

  // check actions for validity
  for (int i = 0; i < int(actions.size()); ++i)
  {
    valid = 1;
    for(size_t j = 0; j < actions[i].size(); ++j)
    {
      ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  %0.3f", i, actions[i][j][0], actions[i][j][1], actions[i][j][2], actions[i][j][3], actions[i][j][4], actions[i][j][5], actions[i][j][6]);

      // check joint limits
      if(!rmodel_->checkJointLimits(actions[i][j]))
        valid = -1;

      //check for collisions
      if(!cc_->isStateValid(actions[i][j], prm_->verbose_collisions_, false, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d  dist: %0.3f is in collision.", i, dist);
        valid = -2;
      }

      if(valid < 1)
        break;
    }

    if(valid < 1)
      continue;

    // check for collisions along path from parent to first waypoint
    if(!cc_->isStateToStateValid(source_angles, actions[i][0], path_length, nchecks, dist))
    {
      ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
      valid = -3;
    }

    if(valid < 1)
      continue;

    // check for collisions between waypoints
    for(size_t j = 1; j < actions[i].size(); ++j)
    {
      //ROS_INFO("[ succ: %d] Checking interpolated path from waypoint %d to waypoint %d.", int(i), int(j-1), int(j));
      if(!cc_->isStateToStateValid(actions[i][j-1], actions[i][j], path_length, nchecks, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
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
   
    //ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", int(i), actions[i].back()[0], actions[i].back()[1], actions[i].back()[2], actions[i].back()[3], actions[i].back()[4], actions[i].back()[5], actions[i].back()[6]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d]   pose: %0.3f %0.3f %0.3f   %0.3f %0.3f %0.3f", int(i), pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d]    xyz: %d %d %d  goal: %d %d %d  (diff: %d %d %d)", int(i), endeff[0], endeff[1], endeff[2], pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2], abs(pdata_.goal_entry->xyz[0] - endeff[0]), abs(pdata_.goal_entry->xyz[1] - endeff[1]), abs(pdata_.goal_entry->xyz[2] - endeff[2]));

    //check if this state meets the goal criteria
    if(isGoalState(pose, pdata_.goal))
    {
      succ_is_goal_state = true;

      for (int k = 0; k < prm_->num_joints_; k++)
        pdata_.goal_entry->coord[k] = scoord[k];

      pdata_.goal_entry->xyz[0] = endeff[0];
      pdata_.goal_entry->xyz[1] = endeff[1];
      pdata_.goal_entry->xyz[2] = endeff[2];
      pdata_.goal_entry->state = actions[i].back();
      pdata_.goal_entry->dist = dist;
    }

    //check if hash entry already exists, if not then create one
    if((succ_entry = getHashEntry(scoord, succ_is_goal_state)) == NULL)
    {
      succ_entry = createHashEntry(scoord, endeff);
      succ_entry->state = actions[i].back();
      succ_entry->dist = dist;

      ROS_DEBUG_NAMED(prm_->expands_log_, "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", succ_entry->stateID, i, int(succ_entry->dist), cost(parent_entry,succ_entry, succ_is_goal_state), GetFromToHeuristic(succ_entry->stateID, pdata_.goal_entry->stateID), succ_entry->xyz[0],succ_entry->xyz[1],succ_entry->xyz[2]);
    }

    //put successor on successor list with the proper cost
    SuccIDV->push_back(succ_entry->stateID);
    CostV->push_back(cost(parent_entry, succ_entry, succ_is_goal_state));
  }

  pdata_.expanded_states.push_back(SourceStateID);
}

void EnvironmentROBARM3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  ROS_ERROR("ERROR in pdata_... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentROBARM3D::AreEquivalent(int StateID1, int StateID2)
{
  ROS_ERROR("ERROR in pdata_... function: AreEquivalent is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  ROS_ERROR("ERROR in pdata_..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentROBARM3D::SetAllPreds(CMDPSTATE* state)
{
  ROS_ERROR("ERROR in pdata_... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

void EnvironmentROBARM3D::printHashTableHist()
{
  int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

  for(int  j = 0; j < pdata_.HashTableSize; j++)
  {
    if((int)pdata_.Coord2StateIDHashTable[j].size() == 0)
      s0++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 50)
      s1++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 100)
      s50++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 200)
      s100++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 300)
      s200++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 400)
      s300++;
    else
      slarge++;
  }
  ROS_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
      s0,s1, s50, s100, s200,s300,slarge);
}

EnvROBARM3DHashEntry_t* EnvironmentROBARM3D::getHashEntry(const std::vector<int> &coord, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return pdata_.goal_entry;

  int binid = getHashBin(coord);

#if DEBUG
  if ((int)pdata_.Coord2StateIDHashTable[binid].size() > 500)
  {
    ROS_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", 
        binid, coord[0], int(pdata_.Coord2StateIDHashTable[binid].size()));
    printHashTableHist();
  }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)pdata_.Coord2StateIDHashTable[binid].size(); ind++)
  {
    int j = 0;

    for(j = 0; j < int(coord.size()); j++)
    {
      if(pdata_.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
        break;
    }

    if (j == int(coord.size()))
      return pdata_.Coord2StateIDHashTable[binid][ind];
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
  HashEntry->stateID = pdata_.StateID2CoordTable.size();

  //insert into the tables
  pdata_.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = getHashBin(HashEntry->coord);

  //insert the entry into the bin
  pdata_.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
  {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

int EnvironmentROBARM3D::cost(EnvROBARM3DHashEntry_t* HashEntry1, EnvROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal)
{
  return prm_->cost_multiplier_;
}

bool EnvironmentROBARM3D::initEnvironment()
{
  // initialize environment data
  pdata_.init();

  //create empty start & goal states
  int endeff[3] = {0};
  std::vector<int> coord(prm_->num_joints_,0);
  pdata_.start_entry = createHashEntry(coord, endeff);
  pdata_.goal_entry = createHashEntry(coord, endeff);

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize BFS
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  bfs_ = new BFS_3D(dimX, dimY, dimZ);

  //set heuristic function pointer
  getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getXYZHeuristic;

  //set 'environment is initialized' flag
  prm_->ready_to_plan_ = true; 
  ROS_INFO("[env] Environment has been initialized.");
  return true;
}

bool EnvironmentROBARM3D::isGoalState(const std::vector<double> &pose, GoalConstraint &goal)
{
  if(goal.type == XYZ_RPY_GOAL)
  {
    if(fabs(pose[0]-goal.pose[0]) <= goal.xyz_tolerance[0] && 
        fabs(pose[1]-goal.pose[1]) <= goal.xyz_tolerance[1] && 
        fabs(pose[2]-goal.pose[2]) <= goal.xyz_tolerance[2])
    {
      //log the amount of time required for the search to get close to the goal
      if(!pdata_.near_goal)
      {
        pdata_.time_to_goal_region = (clock() - pdata_.t_start) / (double)CLOCKS_PER_SEC;
        pdata_.near_goal = true;
        printf("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)\n", pose[0],pose[1],pose[2],goal.xyz_tolerance[0], goal.pose[0], goal.pose[1], goal.pose[2], pdata_.time_to_goal_region,(int)pdata_.expanded_states.size());
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

  num_prims = max_diff / prm_->max_mprim_offset_ + 0.5;
  cost = num_prims * prm_->cost_multiplier_;

  ROS_DEBUG_NAMED("search", "from: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", angles::normalize_angle(from_config[0]),angles::normalize_angle(from_config[1]),angles::normalize_angle(from_config[2]),angles::normalize_angle(from_config[3]),angles::normalize_angle(from_config[4]),angles::normalize_angle(from_config[5]),angles::normalize_angle(from_config[6]));
  ROS_DEBUG_NAMED("search", "  to: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f diff: %0.2f num_prims: %d cost: %d (mprim_size: %0.3f)", to_config[0],to_config[1],to_config[2],to_config[3],to_config[4],to_config[5],to_config[6], max_diff, num_prims, cost, prm_->max_mprim_offset_);
  
  return cost;
}

int EnvironmentROBARM3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)pdata_.StateID2CoordTable.size() 
      || ToStateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif

  //get X, Y for the state
  EnvROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[FromStateID];
  EnvROBARM3DHashEntry_t* ToHashEntry = pdata_.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

bool EnvironmentROBARM3D::setStartConfiguration(const std::vector<double> angles)
{
  double dist=0;
  int x,y,z;
  std::vector<double> pose(6,0);

  if(int(angles.size()) < prm_->num_joints_)
  {
    ROS_ERROR("Start state does not contain enough enough joint positions.");
    return false;
  }

  //get joint positions of starting configuration
  if(!rmodel_->computePlanningLinkFK(angles, pose))
    ROS_WARN("Unable to compute forward kinematics for initial robot state. Attempting to plan anyway.");
  ROS_INFO("[start]             angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]); 
  ROS_INFO("[start] planning_link pose:   xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

  //check joint limits of starting configuration but plan anyway
  if(!rmodel_->checkJointLimits(angles))
    ROS_WARN("Starting configuration violates the joint limits. Attempting to plan anyway.");

  //check if the start configuration is in collision but plan anyway
  if(!cc_->isStateValid(angles, true, false, dist))
  {
    ROS_WARN("[env] The starting configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
  }

  //get arm position in environment
  anglesToCoord(angles, pdata_.start_entry->coord);
  grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
  pdata_.start_entry->xyz[0] = (int)x;
  pdata_.start_entry->xyz[1] = (int)y;
  pdata_.start_entry->xyz[2] = (int)z;
  ROS_INFO("[start]              coord: %d %d %d %d %d %d %d   pose: %d %d %d", pdata_.start_entry->coord[0], pdata_.start_entry->coord[1], pdata_.start_entry->coord[2], pdata_.start_entry->coord[3], pdata_.start_entry->coord[4], pdata_.start_entry->coord[5], pdata_.start_entry->coord[6], x, y, z);
  return true;
}

bool EnvironmentROBARM3D::setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,is_6dof},{x2,y2,z2,r2,p2,y2,is_6dof}...}

  if(!prm_->ready_to_plan_)
  {
    ROS_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    ROS_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }

  pdata_.goal.pose.resize(6,0);
  pdata_.goal.pose[0] = goals[0][0];
  pdata_.goal.pose[1] = goals[0][1];
  pdata_.goal.pose[2] = goals[0][2];
  pdata_.goal.pose[3] = goals[0][3];
  pdata_.goal.pose[4] = goals[0][4];
  pdata_.goal.pose[5] = goals[0][5];
  pdata_.goal.xyz_tolerance[0] = tolerances[0][0];
  pdata_.goal.xyz_tolerance[1] = tolerances[0][1];
  pdata_.goal.xyz_tolerance[2] = tolerances[0][2];
  pdata_.goal.rpy_tolerance[0] = tolerances[0][3];
  pdata_.goal.rpy_tolerance[1] = tolerances[0][4];
  pdata_.goal.rpy_tolerance[2] = tolerances[0][5];
  pdata_.goal.type = goals[0][6];


  // set goal hash entry
  grid_->worldToGrid(goals[0][0], goals[0][1], goals[0][2], pdata_.goal_entry->xyz[0],pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);

  for(int i = 0; i < prm_->num_joints_; i++)
    pdata_.goal_entry->coord[i] = 0;

  ROS_DEBUG_NAMED(prm_->expands_log_, "time: %f", clock() / (double)CLOCKS_PER_SEC);
  ROS_DEBUG_NAMED(prm_->expands_log_, "A new goal has been set.");
  ROS_DEBUG_NAMED(prm_->expands_log_, "grid: %d %d %d (cells)  xyz: %.2f %.2f %.2f (meters)  (tol: %.3f) rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3f)", pdata_.goal_entry->xyz[0],pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2], pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2], pdata_.goal.xyz_tolerance[0], pdata_.goal.pose[3], pdata_.goal.pose[4], pdata_.goal.pose[5], pdata_.goal.rpy_tolerance[0]);


  // push obstacles into bfs grid
  ros::WallTime start = ros::WallTime::now();
  start = ros::WallTime::now();
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  int walls=0;
  for (int z = 0; z < dimZ - 2; z++)
    for (int y = 0; y < dimY - 2; y++)
      for (int x = 0; x < dimX - 2; x++)
        if(grid_->getDistance(x,y,z) <= prm_->planning_link_sphere_radius_)
        {
          bfs_->setWall(x + 1, y + 1, z + 1); //, true);
          walls++;
        }
  double set_walls_time = (ros::WallTime::now() - start).toSec();
  ROS_INFO("[env] %0.5fsec to set walls in new bfs. (%d walls (%0.3f percent))", set_walls_time, walls, double(walls)/double(dimX*dimY*dimZ));

  /*
  start = ros::WallTime::now();
  ROS_ERROR("sphere radius: %0.3fm  %dcells", prm_->planning_link_sphere_radius_, int(prm_->planning_link_sphere_radius_/grid_->getResolution() + 0.5));
  setDistanceField(bfs_, grid_->getDistanceFieldPtr(), int(prm_->planning_link_sphere_radius_/grid_->getResolution() + 0.5));
  ROS_INFO("[env] %0.5fsec to set walls in bfs.", (ros::WallTime::now() - start).toSec());
  */
  if((pdata_.goal_entry->xyz[0] < 0) || (pdata_.goal_entry->xyz[1] < 0) || (pdata_.goal_entry->xyz[2] < 0))
  {
    ROS_ERROR("Goal is out of bounds. Can't run BFS with {%d %d %d} as start.", pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);
    return false;
  }
  bfs_->run(pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);
  //bfs_->configure(pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);

  pdata_.near_goal = false; 
  pdata_.t_start = clock();
  return true;
}

void EnvironmentROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvROBARM3DHashEntry_t* HashEntry = pdata_.StateID2CoordTable[stateID];

  if(stateID == pdata_.goal_entry->stateID)
    coordToAngles(pdata_.goal_entry->coord, angles);
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
  std::vector<double> angles(prm_->num_joints_,0);

  if(bGoal)
    coordToAngles(pdata_.goal_entry->coord, angles);
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

void EnvironmentROBARM3D::getExpandedStates(std::vector<std::vector<double> >* states)
{
  std::vector<double> angles(prm_->num_joints_,0);
  std::vector<double> state(7,0); // {x,y,z,r,p,y,heur}

  for(size_t i = 0; i < pdata_.expanded_states.size(); ++i)
  {
    StateID2Angles(pdata_.expanded_states[i],angles);
    rmodel_->computePlanningLinkFK(angles,state);
    state[6] = pdata_.StateID2CoordTable[pdata_.expanded_states[i]]->heur;
    states->push_back(state);
    ROS_DEBUG("[%d] id: %d  xyz: %0.2f %0.2f %0.2f", int(i), pdata_.expanded_states[i], state[0], state[1], state[2]);
  }
}

void EnvironmentROBARM3D::computeCostPerCell()
{
  ROS_ERROR("Fill in function to computeCostPerCell()");
  prm_->cost_per_cell_ = 100;

  /*
  int largest_action=0;
  double gridcell_size, eucl_dist, max_dist = 0;
  std::vector<double> pose(6,0), start_pose(6,0), angles(prm_->num_joints_,0), start_angles(prm_->num_joints_,0);

  gridcell_size = grid_->getResolution();

  //starting at zeroed angles, find end effector position after each action
  rmodel_->computePlanningLinkFK(start_angles, start_pose);

  //iterate through all possible actions and find the one with the minimum cost per cell
  for (int i = 0; i < prm_->num_mprims_; i++)
  {
    for(int j = 0; j < prm_->num_joints_; j++)
      angles[j] = DEG2RAD(prm_->mprims_[i][j]);

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

  prm_->setCellCost(int(prm_->cost_multiplier_ / (max_dist/gridcell_size)));

  prm_->cost_per_meter_ = int(prm_->cost_per_cell_ / gridcell_size);

  ROS_INFO("[env] max_dist_traveled_per_smp: %0.3fm  cost per cell: %d  cost per meter: %d  (type: jointspace)", max_dist, prm_->cost_per_cell_,prm_->cost_per_meter_);
  */
}

int EnvironmentROBARM3D::getBFSCostToGoal(int x, int y, int z) const
{
  if(bfs_->getDistance(x,y,z) > 1000000)
    return INT_MAX;
  else
    return int(bfs_->getDistance(x,y,z)) * prm_->cost_per_cell_;
}

int EnvironmentROBARM3D::getXYZHeuristic(int FromStateID, int ToStateID)
{
  EnvROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[FromStateID];

  //get distance heuristic
  if(prm_->use_bfs_heuristic_)
    FromHashEntry->heur = getBFSCostToGoal(FromHashEntry->xyz[0], FromHashEntry->xyz[1], FromHashEntry->xyz[2]);
  else
  {
    double x, y, z;
    grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
    FromHashEntry->heur = getEuclideanDistance(x, y, z, pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2]) * prm_->cost_per_meter_;
  }
  return FromHashEntry->heur;
}

bool EnvironmentROBARM3D::convertStateIDPathToJointTrajectory(const std::vector<int> &idpath, trajectory_msgs::JointTrajectory &traj)
{
  if(idpath.empty())
    return false;

  traj.header.frame_id = prm_->planning_frame_;
  traj.joint_names = prm_->planning_joints_;
  traj.points.resize(idpath.size());
  
  std::vector<double> angles;
  for(size_t i = 0; i < idpath.size(); ++i)
  {
    traj.points[i].positions.resize(prm_->num_joints_);
    StateID2Angles(idpath[i], angles);

    for (int p = 0; p < prm_->num_joints_; ++p)
      traj.points[i].positions[p] = angles::normalize_angle(angles[p]); 
  }

  return true;
}

double EnvironmentROBARM3D::getDistanceToGoal(double x, double y, double z)
{
  double dist;
  int dx, dy, dz;
  grid_->worldToGrid(x, y, z, dx, dy, dz);

  if(prm_->use_bfs_heuristic_)
    dist = double(bfs_->getDistance(dx, dy, dz)) * grid_->getResolution();
  else
    dist = getEuclideanDistance(x, y, z, pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2]);

  return dist;
}

std::vector<double> EnvironmentROBARM3D::getGoal()
{
  return pdata_.goal.pose;
}

visualization_msgs::MarkerArray EnvironmentROBARM3D::getVisualization(std::string type)
{
  visualization_msgs::MarkerArray ma;

  if(type.compare("bfs_walls") == 0)
  {
    std::vector<geometry_msgs::Point> pnts;
    geometry_msgs::Point p;
    int dimX, dimY, dimZ;
    grid_->getGridSize(dimX, dimY, dimZ);
    for (int z = 0; z < dimZ - 2; z++)
      for (int y = 0; y < dimY - 2; y++)
        for (int x = 0; x < dimX - 2; x++)
          if(bfs_->isWall(x+1, y+1, z+1))
          {
            grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
            pnts.push_back(p);
          }
    if(!pnts.empty())
      ma.markers.push_back(viz::getSpheresMarker(pnts, 0.02, 210, grid_->getReferenceFrame(), "bfs_walls", 0));
  }
  else if(type.compare("bfs_values") == 0)
  {
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;
    int dimX, dimY, dimZ;
    grid_->getGridSize(dimX, dimY, dimZ);
    for (int z = 65; z < 100 - 2; z++)
      for (int y = 50; y < 100 - 2; y++)
        for (int x = 65; x < 100 - 2; x++)
        {
          int d = bfs_->getDistance(x+1, y+1, z+1);
          if(d < 10000)
          {
            grid_->gridToWorld(x, y, z, p.position.x, p.position.y, p.position.z);
            double hue = d/30.0 * 300;
            ma.markers.push_back(viz::getTextMarker(p, boost::lexical_cast<std::string>(d), 0.009, hue, grid_->getReferenceFrame(), "bfs_values", ma.markers.size()));
          }
        }
  }
  else
    ROS_ERROR("No such marker type, '%s'.", type.c_str());

  return ma;
}

}
