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
/** \author Benjamin Cohen bcohen@seas.upenn.edu */

#include <sbpl_arm_planner/environment_cartrobarm3d.h>

using namespace std;

namespace sbpl_arm_planner {

bool isEqual(const std::vector<int> &v1, const std::vector<int> &v2)
{
  if(v1.size() != v2.size())
    return false;
  for(size_t i = 0; i < v1.size(); ++i)
  {
    if(v1[i] != v2[i])
      return false;
  }
  return true;
}

bool isEqualDouble(const std::vector<double> &v1, const std::vector<double> &v2)
{
  if(v1.size() != v2.size())
    return false;
  for(size_t i = 0; i < v1.size(); ++i)
  {
    if(v1[i] != v2[i])
      return false;
  }
  return true;
}

EnvironmentCARTROBARM3D::EnvironmentCARTROBARM3D(OccupancyGrid *grid, SBPLKinematicModel *kmodel, CollisionChecker *cc)
{
  grid_ = grid;
  kmodel_ = kmodel;
  cc_ = cc;

  EnvROBARMCfg.bInitialized = false;
  save_expanded_states = true;
  using_short_mprims_ = false;
  free_angle_index_ = 2;
  ndof_ = 7;

  mp_dist_.resize(6);
  mp_gradient_.resize(3);

  prms_.environment_type_ = "cartesian";
}

/*
EnvironmentCARTROBARM3D::~EnvironmentCARTROBARM3D()
{
  if(rpysolver_ != NULL)
    delete rpysolver_;

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
*/

void EnvironmentCARTROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  GetSuccs(SourceStateID,SuccIDV,CostV,NULL);
}

void EnvironmentCARTROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV)
{
  bool invalid_prim = false;
  double dist=0, dist_temp=0;
  size_t i=0, a=0, j=0, q=0;
  int motion_cost=0, motion_code=0, convert_code=0, nchecks=0, path_length=0, wp_length=0;
  std::vector<int> succcoord(ndof_,0);
  std::vector<int> mp_coord(ndof_,0);
  std::vector<double> zeros(ndof_,0);
  std::vector<double> parent_angles(ndof_,0), sangles(ndof_,0), iangles(ndof_,0), succ_wcoord(ndof_,0), init_wcoord(ndof_,0), final_wcoord(ndof_,0);
  int xyz[3]={0},rpy[3]={0},fa=0;
  double wxyz[3]={0},wrpy[3]={0},wfa=0, xyz_source[3]={0},rpy_source[3]={0},fa_source=0;
  std::vector<std::vector<double> > wptraj;

  double mind, avgd;

  // clear the successor arrays
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(prms_.mp_.size());
  CostV->reserve(prms_.mp_.size());
  if(ActionV != NULL)
  {
    ActionV->clear();
    ActionV->reserve(prms_.mp_.size());
  }

  // goal state should be absorbing
  if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    return;

  EnvROBARM3DHashEntry_t* parent = EnvROBARM.StateID2CoordTable[SourceStateID];

  // default coords of successor
  for(i = 0; i < parent->coord.size(); i++)
    succcoord[i] = parent->coord[i];

  for(size_t p = 0; p < parent->angles.size(); ++p)
    parent_angles[p] = parent->angles[p];

  //int xyz_heur = getBFSCostToGoal(parent->coord[0], parent->coord[1], parent->coord[2]);
  stateIDToWorldPose(SourceStateID, xyz_source, rpy_source, &fa_source);
  ROS_DEBUG_NAMED(prms_.expands_log_, "[Source: %d] xyz: %.3f %.3f %.3f  rpy: % 2.3f % 2.3f % 2.3f  fa: % 2.3f  heur: %3d   {[dist to goal] xyz: %2d %2d %2d}",SourceStateID,xyz_source[0],xyz_source[1],xyz_source[2],rpy_source[0],rpy_source[1],rpy_source[2],fa_source, parent->heur, EnvROBARM.goalHashEntry->xyz[0] - parent->coord[0], EnvROBARM.goalHashEntry->xyz[1] - parent->coord[1], EnvROBARM.goalHashEntry->xyz[2] - parent->coord[2]);

  // iterate through successors of source state
  for (i = 0; i < prms_.mp_.size(); i++)
  {
    // should we use this mprim or skip it?
    if(!getMotionPrimitive(parent, prms_.mp_[i]))
    {
      ROS_DEBUG_NAMED(prms_.expands2_log_, "  skipping motion primitive: %s  (dist: %d)", prms_.motion_primitive_type_names_[int(prms_.mp_[i].type)].c_str(), parent->heur);
      continue;
    }
    stats_.updateExpand();

    dist = 100;
    invalid_prim = false;
    motion_cost = 0;

    // succcoord turns into the coords of the successor
    for(q = 0; q < parent->coord.size(); ++q)
      succcoord[q] = parent->coord[q];

    // iangles is the arm configuration between intermediate waypoints
    iangles = parent_angles;

    // init_wcoord is the world coords between intermediate waypoints    
    coordToWorldPose(parent->coord, init_wcoord);

    // the mp is subject to change (e.g. adaptive, ik search)
    mp_coord = prms_.mp_[i].coord;

    //get the successor
    EnvROBARM3DHashEntry_t* OutHashEntry;
    bool bSuccisGoal = false;

    // loop through the waypoints in the motion primitive
    for(j = 0; j < prms_.mp_[i].m.size(); ++j)
    {
      if(isEqualDouble(prms_.mp_[i].m[j], zeros))
      {
        //ROS_DEBUG_NAMED(prms_.expands_log_, "  succ: %2d-%1d       mp_type: %10s   MP SUB-WAYPOINT IS ALL ZEROS", int(i), int(j), prms_.motion_primitive_type_names_[prms_.mp_[i].type].c_str());
        continue;
      }

      if((convert_code = getJointAnglesForMotionPrimWaypoint(prms_.mp_[i].m[j], init_wcoord, iangles, final_wcoord, wptraj)) < 0)
      {
        ROS_DEBUG_NAMED(prms_.expands_log_, "  succ: %2d-%1d       mp_type: %10s  convert_code: %2d  Converting to angles failed.", int(i), int(j), prms_.motion_primitive_type_names_[prms_.mp_[i].type].c_str(), convert_code);
        invalid_prim = true;
        break;
      }
      else
      {
        stats_.total_num_solver_used_[convert_code]++;
        wp_length = 0;

        if(isEqualDouble(init_wcoord, final_wcoord))
          ROS_ERROR("  succ: %2d-%1d WTF...these world coords should not be equal.", int(i), int(j));

        for(q = 0; q < wptraj.size(); ++q)
        {
          // debug - remove this eventually
          if(isEqualDouble(wptraj[q], iangles))
            ROS_ERROR("  succ: %2d  waypoint: %1d  sub-waypoint: %2d(%d)  convert_code: %2d   ** Has the same angles as its parent.**", int(i), int(j), int(q), int(wptraj.size()), convert_code);
        
          //ROS_DEBUG_NAMED(prms_.expands_log_, "    [%d] -> [%d-%d-%d] sub-waypoint: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f ", SourceStateID, int(i), int(j), int(q), wptraj[q][0],wptraj[q][1],wptraj[q][2],wptraj[q][3],wptraj[q][4],wptraj[q][5],wptraj[q][6]);

          ROS_ERROR("Not getting clearance anymore...");
          /*
          if(!cspace_->getClearance(wptraj[q], 8, avgd, mind))
            ROS_ERROR("Failed at getClearance");
          */

          // check motion for collisions
          if((motion_code = cc_->isStateToStateValid(iangles, wptraj[q], path_length, nchecks, dist_temp)) < 0)
          {
            stats_.nchecks_per_solver_per_succ_[convert_code] += nchecks;
            ROS_DEBUG_NAMED(prms_.expands_log_, "  succ: %2d-%1d-%1d(%1d)  mp_type: %10s  convert_code: %2d  valid_code: %2d  dist: %3d  # checks: %2d(%2d)    Invalid.", int(i), int(j), int(q), int(wptraj.size()), prms_.motion_primitive_type_names_[prms_.mp_[i].type].c_str(),  convert_code, motion_code, int(dist_temp), stats_.nchecks_per_solver_per_succ_[convert_code], path_length);
            dist = dist_temp;
            invalid_prim = true;
            break;
          }
          stats_.nchecks_per_solver_per_succ_[convert_code] += nchecks;
          wp_length += path_length;

          if(dist_temp < dist)
            dist = dist_temp;

          //save joint angles of current waypoint for next sub-waypoint in intermediate waypoint
          iangles = wptraj[q];
        }
   
        if(invalid_prim)
          break;
        
        ROS_DEBUG_NAMED(prms_.expands_log_, "  succ: %2d-%1d-%1d     mp_type: %10s  convert_code: %2d  valid_code: %2d  dist: %3d  # checks: %2d(%2d)    Valid.", int(i), int(j), int(wptraj.size()), prms_.motion_primitive_type_names_[prms_.mp_[i].type].c_str(), convert_code, motion_code, int(dist_temp), stats_.nchecks_per_solver_per_succ_[convert_code], wp_length);
        /*
        if(prms_.mp_[i].type == SNAP_TO_RPY_AT_START)
        {
           ROS_DEBUG_NAMED(prms_.expands_log_, "  succ: %2d-%1d-%1d     mp_type: %10s  convert_code: %2d  valid_code: %2d  dist: %3d  # checks: %2d(%2d)    Valid.", int(i), int(j), int(wptraj.size()), prms_.motion_primitive_type_names_[prms_.mp_[i].type].c_str(), convert_code, motion_code, int(dist_temp), stats_.nchecks_per_solver_per_succ_[convert_code], wp_length);
           aviz_->visualizePose(init_wcoord, "adapmp-parent");
           aviz_->visualizePose(final_wcoord, "adapmp-succ");
           sleep(0.25);
        }
        */
        // update current joint angle vector (sangles)
        sangles = wptraj.back();

        // update the current world coords
        init_wcoord = final_wcoord;

        // if the FA was changed by the IK search and it is the final waypoint of the mp
        if(convert_code == 3 && j == prms_.mp_[i].m.size()-1)
        {
          int fa;
          worldToDiscFAngle(sangles[2], &fa);
          mp_coord[6] = parent->coord[6] - fa;
          ROS_DEBUG("  IK search succeeded. Updated FA in coord: %d %d %d %d %d %d %d -> %d %d %d %d %d %d %d", prms_.mp_[i].coord[0],prms_.mp_[i].coord[1],prms_.mp_[i].coord[2],prms_.mp_[i].coord[3],prms_.mp_[i].coord[4],prms_.mp_[i].coord[5],prms_.mp_[i].coord[6],mp_coord[0],mp_coord[1],mp_coord[2],mp_coord[3],mp_coord[4],mp_coord[5],mp_coord[6]);
        }

        if(dist_temp < dist)
          dist = dist_temp;

        // add up motion cost for all waypoints in mprim
        motion_cost += computeMotionCost(iangles, sangles);
      }
    }

    unsigned char dg = int((avgd+0.5)/prms_.xyz_resolution_);
    computeGradient(prms_.mp_[i], dg, invalid_prim);

    if(invalid_prim)
      continue;

    //compute the successor coords
    for(a = 0; a < mp_coord.size(); ++a)
    { 
      if((succcoord[a] + mp_coord[a]) < 0)
        succcoord[a] = ((EnvROBARMCfg.coord_vals[a] + succcoord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
      else
        succcoord[a] = ((int)(succcoord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
    }
    /*
    if(prms_.mp_[i].type == SNAP_TO_RPY_AT_START)
    {
      ROS_DEBUG_NAMED(prms_.expands_log_,"   [%d]    parent: %2d %2d %2d %2d %2d %2d %2d  (heur: %4d)", int(i), parent->coord[0], parent->coord[1], parent->coord[2], parent->coord[3], parent->coord[4], parent->coord[5], parent->coord[6], dijkstra_->getDist(parent->coord[0], parent->coord[1], parent->coord[2]));
      ROS_DEBUG_NAMED(prms_.expands_log_,"   [%d]      succ: %2d %2d %2d %2d %2d %2d %2d  (heur: %4d)", int(i), succcoord[0], succcoord[1], succcoord[2], succcoord[3], succcoord[4], succcoord[5], succcoord[6], dijkstra_->getDist(succcoord[0], succcoord[1], succcoord[2]));
      ROS_DEBUG_NAMED(prms_.expands_log_,"   [%d]  mp-coord: %2d %2d %2d %2d %2d %2d %2d", int(i), mp_coord[0], mp_coord[1], mp_coord[2], mp_coord[3], mp_coord[4], mp_coord[5], mp_coord[6]);
      ROS_DEBUG_NAMED(prms_.expands_log_,"   [%d]      diff: %2d %2d %2d %2d %2d %2d %2d", int(i), succcoord[0]-parent->coord[0], succcoord[1]-parent->coord[1], succcoord[2]-parent->coord[2], succcoord[3]-parent->coord[3], succcoord[4]-parent->coord[4], succcoord[5]-parent->coord[5], succcoord[6]-parent->coord[6]);
      ROS_DEBUG_NAMED(prms_.expands_log_,"   [%d] goal-diff: %2d %2d %2d %2d %2d %2d %2d", int(i), EnvROBARM.goalHashEntry->coord[0]-succcoord[0], EnvROBARM.goalHashEntry->coord[1]-succcoord[1], EnvROBARM.goalHashEntry->coord[2]-succcoord[2], EnvROBARM.goalHashEntry->coord[3]-succcoord[3], EnvROBARM.goalHashEntry->coord[4]-succcoord[4], EnvROBARM.goalHashEntry->coord[5]-succcoord[5], EnvROBARM.goalHashEntry->coord[6]-succcoord[6]);
    }
    */

    /* write a function that makes this conversion smarter */
    coordToPose(succcoord, xyz, rpy, &fa);
    discToWorldXYZ(xyz,wxyz);
    discToWorldRPY(rpy,wrpy);
    discToWorldFAngle(fa,&wfa);

    //check if this state meets the goal criteria
    if(isGoalPosition(wxyz, wrpy, wfa))
    {
      bSuccisGoal = true;

      for (int j = 0; j < ndof_; j++)
        EnvROBARM.goalHashEntry->coord[j] = succcoord[j];
      
      EnvROBARM.goalHashEntry->dist = dist;
      EnvROBARM.goalHashEntry->angles = sangles;
      ROS_DEBUG("[search] Goal state has been found. Parent StateID: %d (obstacle distance: %d)",SourceStateID, int(dist));
      ROS_DEBUG("[search]   coord: %d %d %d %d %d %d %d", succcoord[0], succcoord[1],succcoord[2],succcoord[3],succcoord[4],succcoord[5],succcoord[6]);
    }

    //check if hash entry already exists, if not then create one
    if((OutHashEntry = getHashEntry(succcoord, bSuccisGoal)) == NULL)
    {
      OutHashEntry = createHashEntry(succcoord);
      OutHashEntry->dist = dist;
      OutHashEntry->angles = sangles;
      OutHashEntry->action = i;
      ROS_DEBUG("  parentid: %d  stateid: %d  mprim: %d  cost: %4d  heur: %2d  xyz: %3d %3d %3d  rpy: %3d %3d %3d  fa: %3d", SourceStateID, OutHashEntry->stateID, int(i),  motion_cost, GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID), OutHashEntry->coord[0],OutHashEntry->coord[1],OutHashEntry->coord[2], OutHashEntry->coord[3],OutHashEntry->coord[4],OutHashEntry->coord[5],OutHashEntry->coord[6]);
      if(GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID) > 100000)
      {
        ROS_DEBUG_NAMED(prms_.expands_log_, "  parentid: %d  stateid: %d  mprim: %d  xyz: %3d %3d %3d  rpy: %3d %3d %3d  fa: %3d  has an INFINITE HEURISTIC", SourceStateID, OutHashEntry->stateID, int(i), OutHashEntry->coord[0],OutHashEntry->coord[1],OutHashEntry->coord[2], OutHashEntry->coord[3],OutHashEntry->coord[4],OutHashEntry->coord[5],OutHashEntry->coord[6]);
      }
    }

    if(prms_.mp_[i].type == sbpl_arm_planner::ADAPTIVE)
      motion_cost = 30;
    else
      motion_cost = 60;

    //put successor on successor list with the proper cost
    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(motion_cost); //(cost(parent,OutHashEntry,bSuccisGoal));
    if(ActionV != NULL)
      ActionV->push_back(i);
  }

  stats_.updateExpand();
  //stats_.printChecksPerSolverPerSingleExpand();
  stats_.logChecksForExpand();
  if(save_expanded_states)
    expanded_states.push_back(SourceStateID);
}

EnvROBARM3DHashEntry_t* EnvironmentCARTROBARM3D::getHashEntry(const std::vector<int> &coord, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return EnvROBARM.goalHashEntry;

  int binid = getHashBin(coord);

  //iterate over the states in the bin and select the perfect match
  for(size_t ind = 0; ind < EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++)
  {
    size_t j = 0;
    
    for(j=0; j<coord.size(); j++)
    {
      if(EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j])
        break;
    }

    if (j == 7)
      return EnvROBARM.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvROBARM3DHashEntry_t* EnvironmentCARTROBARM3D::getHashEntry(int *xyz, int *rpy, int fangle, bool bIsGoal)
{
  std::vector<int> coord(ndof_,0);
  coord[0] = xyz[0];
  coord[1] = xyz[1];
  coord[2] = xyz[2];
  coord[3] = rpy[0];
  coord[4] = rpy[1];
  coord[5] = rpy[2];
  coord[6] = fangle;

  return getHashEntry(coord,bIsGoal);
}

EnvROBARM3DHashEntry_t* EnvironmentCARTROBARM3D::createHashEntry(const std::vector<int> &coord)
{
  int i=0;

  EnvROBARM3DHashEntry_t* HashEntry = new EnvROBARM3DHashEntry_t;

  HashEntry->coord = coord;
 
  HashEntry->dist = 200.0;

  HashEntry->angles.resize(ndof_,-1);

  HashEntry->xyz[0] = coord[0];
  HashEntry->xyz[1] = coord[1];
  HashEntry->xyz[2] = coord[2];

  //assign a stateID to HashEntry to be used 
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
  for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    StateID2IndexMapping[HashEntry->stateID][i] = -1;

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

bool EnvironmentCARTROBARM3D::initEnvConfig()
{
  std::vector<int> coord(ndof_,0);

  //CARTTODO
  EnvROBARMCfg.coord_delta.resize(ndof_,1);
  EnvROBARMCfg.coord_vals.resize(ndof_,360);

  EnvROBARMCfg.coord_delta[0] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[1] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[2] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[3] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[4] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[5] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[6] = prms_.fa_resolution_;

  EnvROBARMCfg.coord_vals[0] = prms_.sizeX_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[1] = prms_.sizeY_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[2] = prms_.sizeZ_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[3] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[4] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[5] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[6] = (2.0*M_PI) / prms_.fa_resolution_ + 0.5;

  ROS_INFO("[env] Discretization of Cartesian Statespace:");
  for(int i = 0; i < ndof_; ++i)
    ROS_INFO("[env] [%d] delta: %0.3f  vals: %d",i, EnvROBARMCfg.coord_delta[i],EnvROBARMCfg.coord_vals[i]);

  //initialize the map from Coord to StateID
  EnvROBARM.HashTableSize = 32*1024; //should be power of two
  EnvROBARM.Coord2StateIDHashTable = new std::vector<EnvROBARM3DHashEntry_t*>[EnvROBARM.HashTableSize];

  //initialize the map from StateID to Coord
  EnvROBARM.StateID2CoordTable.clear();

  //create empty start & goal states
  EnvROBARM.startHashEntry = createHashEntry(coord);
  EnvROBARM.goalHashEntry = createHashEntry(coord);

  return true;
}

bool EnvironmentCARTROBARM3D::isGoalPosition(double *xyz, double *rpy, double fangle)
{
  //check position
  if(fabs(xyz[0]-EnvROBARMCfg.goal.xyz[0]) <= EnvROBARMCfg.goal.xyz_tolerance[0] &&
      fabs(xyz[1]-EnvROBARMCfg.goal.xyz[1]) <= EnvROBARMCfg.goal.xyz_tolerance[1] &&
      fabs(xyz[2]-EnvROBARMCfg.goal.xyz[2]) <= EnvROBARMCfg.goal.xyz_tolerance[2])
  {
    //log the amount of time required for the search to get close to the goal
    if(!near_goal && save_expanded_states)
    {
      time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
      near_goal = true;
      ROS_INFO("[env] Search is within %0.3f meters of the goal (%0.3f %0.3f %0.3f) after %.4f sec. (after %d expansions)", EnvROBARMCfg.goal.xyz_tolerance[0], EnvROBARMCfg.goal.xyz[0], EnvROBARMCfg.goal.xyz[1], EnvROBARMCfg.goal.xyz[2], time_to_goal_region, (int)expanded_states.size());
    }
    //check orientation
    if (fabs(angles::shortest_angular_distance(rpy[0], EnvROBARMCfg.goal.rpy[0])) <= EnvROBARMCfg.goal.rpy_tolerance[0] &&
        fabs(angles::shortest_angular_distance(rpy[1], EnvROBARMCfg.goal.rpy[1])) <= EnvROBARMCfg.goal.rpy_tolerance[1] &&
        fabs(angles::shortest_angular_distance(rpy[2], EnvROBARMCfg.goal.rpy[2])) <= EnvROBARMCfg.goal.rpy_tolerance[2])
      return true;

    ROS_INFO("[env] xyz goal is met. rpy isn't. rpy: %0.3f %0.3f %0.3f  diff: %1.3f %1.3f %1.3f  tolerance: %1.3f %1.3f %1.3f", rpy[0],rpy[1],rpy[2], fabs(angles::shortest_angular_distance(rpy[0], EnvROBARMCfg.goal.rpy[0])), fabs(angles::shortest_angular_distance(rpy[1], EnvROBARMCfg.goal.rpy[1])), fabs(angles::shortest_angular_distance(rpy[2], EnvROBARMCfg.goal.rpy[2])), EnvROBARMCfg.goal.rpy_tolerance[0], EnvROBARMCfg.goal.rpy_tolerance[1], EnvROBARMCfg.goal.rpy_tolerance[2]);
  }
  return false;
}

bool EnvironmentCARTROBARM3D::setStartConfiguration(std::vector<double> angles)
{
  //getHeuristic_ = &sbpl_arm_planner::EnvironmentROBARM3D::getXYZRPYHeuristic;

  if(int(angles.size()) < num_joints_)
  {
    ROS_WARN("[env] Failed to set start configuration. Not enough joint positions. (expected %d, received %d)", num_joints_, int(angles.size()));
    return false;
  }

  //check joint limits of starting configuration but plan anyway
  if(!kmodel_->checkJointLimits(angles))
    ROS_WARN("[env] Starting configuration violates the joint limits. Attempting to plan anyway.");

  double dist = 100;
  if(!cc_->isStateValid(angles, true, false, dist))
    ROS_WARN("[env] Starting configuration is in collision. (dist: %d)", int(dist));
  else
    ROS_INFO("[env] Starting configuration is valid. (dist: %d)", int(dist));

  //set start position
  anglesToCoord(angles, EnvROBARM.startHashEntry->coord);
  EnvROBARM.startHashEntry->angles = angles;
  EnvROBARM.startHashEntry->xyz[0] = EnvROBARM.startHashEntry->coord[0];
  EnvROBARM.startHashEntry->xyz[1] = EnvROBARM.startHashEntry->coord[1];
  EnvROBARM.startHashEntry->xyz[2] = EnvROBARM.startHashEntry->coord[2];

  ROS_INFO("[env] [start_state] xyz: %d %d %d rpy: %d %d %d angle: %d",EnvROBARM.startHashEntry->coord[0],EnvROBARM.startHashEntry->coord[1],EnvROBARM.startHashEntry->coord[2],EnvROBARM.startHashEntry->coord[3],EnvROBARM.startHashEntry->coord[4],EnvROBARM.startHashEntry->coord[5],EnvROBARM.startHashEntry->coord[6]);
  return true;
}

bool EnvironmentCARTROBARM3D::setGoalPosition(const std::vector<std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,fangle,is_6dof},...}

  if(!EnvROBARMCfg.bInitialized)
  {
    ROS_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    ROS_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }

  // debugging - check if an IK solution exists for the goal pose before we do the search
  // we plan even if there is no solution
  std::vector<double> pose(7,0), jnt_angles(7,0), ik_solution(7,0);
  pose = goals[0];
  if(!kmodel_->computeIK(pose, jnt_angles, ik_solution))
    ROS_WARN("[setGoalPosition] No valid IK solution for the goal pose.");

  // only supports a single goal
  EnvROBARMCfg.goal.xyz[0] = goals[0][0];
  EnvROBARMCfg.goal.xyz[1] = goals[0][1];
  EnvROBARMCfg.goal.xyz[2] = goals[0][2];
  EnvROBARMCfg.goal.rpy[0] = goals[0][3];
  EnvROBARMCfg.goal.rpy[1] = goals[0][4];
  EnvROBARMCfg.goal.rpy[2] = goals[0][5];
  EnvROBARMCfg.goal.fangle = goals[0][6];

  EnvROBARMCfg.goal.xyz_tolerance[0] = tolerances[0][0];
  EnvROBARMCfg.goal.xyz_tolerance[1] = tolerances[0][0];
  EnvROBARMCfg.goal.xyz_tolerance[2] = tolerances[0][0];  // TODO confirm this
  EnvROBARMCfg.goal.rpy_tolerance[0] = tolerances[0][3];
  EnvROBARMCfg.goal.rpy_tolerance[1] = tolerances[0][4];
  EnvROBARMCfg.goal.rpy_tolerance[2] = tolerances[0][5];

  EnvROBARMCfg.goal.xyz_disc_tolerance = tolerances[0][0] / grid_->getResolution();
  EnvROBARMCfg.goal.rpy_disc_tolerance = tolerances[0][3] / EnvROBARMCfg.coord_delta[3];

  EnvROBARMCfg.goal.type = goals[0][7];
  prms_.use_6d_pose_goal_ = goals[0][7];

  worldPoseToCoord(EnvROBARMCfg.goal.xyz, EnvROBARMCfg.goal.rpy, EnvROBARMCfg.goal.fangle, EnvROBARM.goalHashEntry->coord);
  EnvROBARMCfg.goal.xyz_disc[0] = EnvROBARM.goalHashEntry->coord[0];
  EnvROBARMCfg.goal.xyz_disc[1] = EnvROBARM.goalHashEntry->coord[1];
  EnvROBARMCfg.goal.xyz_disc[2] = EnvROBARM.goalHashEntry->coord[2];
  EnvROBARMCfg.goal.rpy_disc[0] = EnvROBARM.goalHashEntry->coord[3];
  EnvROBARMCfg.goal.rpy_disc[1] = EnvROBARM.goalHashEntry->coord[4];
  EnvROBARMCfg.goal.rpy_disc[2] = EnvROBARM.goalHashEntry->coord[5];

  EnvROBARM.goalHashEntry->xyz[0] = EnvROBARMCfg.goal.xyz_disc[0];
  EnvROBARM.goalHashEntry->xyz[1] = EnvROBARMCfg.goal.xyz_disc[1];
  EnvROBARM.goalHashEntry->xyz[2] = EnvROBARMCfg.goal.xyz_disc[2];

  if(!prms_.use_6d_pose_goal_)
    ROS_DEBUG("[setGoalPosition] Goal position constraint set. No goal orientation constraint requested.\n");

  ROS_INFO("[goal]");
  ROS_INFO(" xyz: %.2f %.2f %.2f (meters) (tol: %.3fm)", EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1],EnvROBARMCfg.goal.xyz[2],EnvROBARMCfg.goal.xyz_tolerance[0]);
  ROS_INFO(" rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3frad)", EnvROBARMCfg.goal.rpy[0],EnvROBARMCfg.goal.rpy[1],EnvROBARMCfg.goal.rpy[2],EnvROBARMCfg.goal.rpy_tolerance[0]);
  ROS_INFO(" coord: %u %u %u (tol: %d)   %u %u %u (tol: %d)", EnvROBARM.goalHashEntry->coord[0], EnvROBARM.goalHashEntry->coord[1], EnvROBARM.goalHashEntry->coord[2], EnvROBARMCfg.goal.xyz_disc_tolerance, EnvROBARM.goalHashEntry->coord[3], EnvROBARM.goalHashEntry->coord[4], EnvROBARM.goalHashEntry->coord[5], EnvROBARMCfg.goal.rpy_disc_tolerance);


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
  ROS_INFO("[env] %0.5fsec to set walls in new bfs. (%d walls (%0.3f percent))", set_walls_time, walls, double(walls)/double(dimX*dimY*dimZ));
  start = ros::WallTime::now();
  bfs_->run(EnvROBARM.goalHashEntry->xyz[0], EnvROBARM.goalHashEntry->xyz[1], EnvROBARM.goalHashEntry->xyz[2]);
  ROS_INFO("[env] COST TO GOAL OF START STATE: %d", getBFSCostToGoal(EnvROBARM.startHashEntry->xyz[0], EnvROBARM.startHashEntry->xyz[1], EnvROBARM.startHashEntry->xyz[2]));
  ROS_INFO("[env] Time required to compute at least enough of the BFS to reach the start state: %0.4fsec", (ros::WallTime::now() - start).toSec());

  stats_.resetSolverCounters();
  stats_.resetAllCheckCounters();
  expanded_states.clear();
  return true;
}

void EnvironmentCARTROBARM3D::stateIDToPose(int stateID, int *xyz, int *rpy, int *fangle)
{
  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
  {
    xyz[0] = EnvROBARM.goalHashEntry->coord[0];
    xyz[1] = EnvROBARM.goalHashEntry->coord[1];
    xyz[2] = EnvROBARM.goalHashEntry->coord[2];
    rpy[0] = EnvROBARM.goalHashEntry->coord[3];
    rpy[1] = EnvROBARM.goalHashEntry->coord[4];
    rpy[2] = EnvROBARM.goalHashEntry->coord[5];
    *fangle = EnvROBARM.goalHashEntry->coord[6];
  }
  else
  {
    xyz[0] = HashEntry->coord[0];
    xyz[1] = HashEntry->coord[1];
    xyz[2] = HashEntry->coord[2];
    rpy[0] = HashEntry->coord[3];
    rpy[1] = HashEntry->coord[4];
    rpy[2] = HashEntry->coord[5];
    *fangle = HashEntry->coord[6];
  }
}

void EnvironmentCARTROBARM3D::stateIDToWorldPose(int stateID, double *xyz, double *rpy, double *fangle)
{
  int dxyz[3]={0}, drpy[3]={0}, dfangle=0;

  stateIDToPose(stateID, dxyz, drpy, &dfangle);

  discToWorldXYZ(dxyz, xyz);
  discToWorldRPY(drpy, rpy);
  discToWorldFAngle(dfangle, fangle);
}

void EnvironmentCARTROBARM3D::worldPoseToCoord(double *wxyz, double *wrpy, double wfangle, std::vector<int> &coord)
{
  int xyz[3]={0}, rpy[3]={0}, fangle=0;

  worldToDiscXYZ(wxyz, xyz);
  worldToDiscRPY(wrpy, rpy);
  worldToDiscFAngle(wfangle, &fangle);
  
  coord.resize(ndof_);
  coord[0] = xyz[0];
  coord[1] = xyz[1];
  coord[2] = xyz[2];
  coord[3] = rpy[0];
  coord[4] = rpy[1];
  coord[5] = rpy[2];
  coord[6] = fangle;
}

void EnvironmentCARTROBARM3D::worldPoseToState(double *wxyz, double *wrpy, double wfangle, bool is_goal, EnvROBARM3DHashEntry_t *state)
{
  std::vector<int> coord(7,0);

  worldPoseToCoord(wxyz,wrpy,wfangle,coord);

  if((state = getHashEntry(coord,is_goal)) == NULL)
    state = createHashEntry(coord);
  else
    state->coord = coord;
}

void EnvironmentCARTROBARM3D::anglesToCoord(const std::vector<double> &angles, std::vector<int> &coord)
{
  std::vector<double> pose(6,0);
  
  kmodel_->computePlanningLinkFK(angles, pose);
  
  double fangle=angles[free_angle_index_];
  double wxyz[3] = {pose[0], pose[1], pose[2]};
  double wrpy[3] = {pose[3], pose[4], pose[5]};

  worldPoseToCoord(wxyz,wrpy,fangle,coord);
}

void EnvironmentCARTROBARM3D::computeCostPerCell()
{
  int cost_per_cell = prms_.cost_per_second_ * prms_.time_per_cell_;
  prms_.setCellCost(cost_per_cell);
  prms_.cost_per_meter_ = int(prms_.cost_per_cell_ / prms_.xyz_resolution_);
  ROS_INFO("[env] cost per cell: %d, time per cell: %0.3fsec  (type: cartesian)", cost_per_cell, prms_.time_per_cell_);
}

int EnvironmentCARTROBARM3D::computeMotionCost(const std::vector<double> &a, const std::vector<double> &b)
{
  double time = 0, time_max = 0;

  for(size_t i = 0; i < a.size(); ++i)
  {
    time = fabs(angles::normalize_angle(a[i]-b[i])) / prms_.joint_vel_[i];

   ROS_DEBUG("%d: a: %0.4f b: %0.4f dist: %0.4f vel:%0.4f time: %0.4f", int(i), a[i], b[i], fabs(angles::normalize_angle(a[i]-b[i])), prms_.joint_vel_[i], time);
 
   if(time > time_max)
     time_max = time;
  }

  ROS_DEBUG("motion cost: %d  max_time:%0.4f",  int(prms_.cost_per_second_ * time_max), time_max);

  return prms_.cost_per_second_ * time_max;
}

bool EnvironmentCARTROBARM3D::convertCoordToAngles(const std::vector<int> *coord, std::vector<double> *angles)
{
  //ROS_INFO("[convertCoordToAngles] Converting:  xyz: %u %u %u  rpy: %u %u %u  a: %u",coord->at(0),coord->at(1),coord->at(2),coord->at(3),coord->at(4),coord->at(5),coord->at(6));

  double wxyz[3]={0}, wrpy[3]={0}, wfangle=0;
  std::vector<double> pose(6,0), seed(7,0);

  coordToWorldPose(*coord,wxyz,wrpy,&wfangle);
  pose[0] = wxyz[0];
  pose[1] = wxyz[1];
  pose[2] = wxyz[2];
  pose[3] = wrpy[0];
  pose[4] = wrpy[1];
  pose[5] = wrpy[2];
  seed[free_angle_index_] = wfangle;

  if(!kmodel_->computeFastIK(pose,seed,*angles))
  {
    ROS_DEBUG("computeFastIK failed to return a solution");

    /*
    if(!arm_->computeIK(pose, seed, angles))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
    */
  }

  return true;
}

bool EnvironmentCARTROBARM3D::convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> &angles)
{
  std::vector<double> pose(6,0), seed(7,0);

  for(size_t i = 0; i < 6; ++i)
    pose[i] = wpose[i];

  seed[free_angle_index_] = wpose[6];

  if(!kmodel_->computeFastIK(pose,seed,angles))
  {
    ROS_DEBUG("computeFastIK failed to return a solution");

    if(!kmodel_->computeIK(pose,seed,angles))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
  }

  return true;
}

bool EnvironmentCARTROBARM3D::convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> seed, std::vector<double> &angles)
{
  std::vector<double> pose(6,0);

  for(size_t i = 0; i < 6; ++i)
    pose[i] = wpose[i];

  seed[free_angle_index_] = wpose[6];

  if(!kmodel_->computeFastIK(pose,seed,angles))
  {
    ROS_DEBUG("computeFastIK failed to return a solution");
    return false;
    /*
    if(!arm_->computeIK(pose,seed,angles))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
    */
  }

  return true;
}

void EnvironmentCARTROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
    angles = EnvROBARM.goalHashEntry->angles;
  else
    angles = HashEntry->angles;

  for (size_t i = 0; i < angles.size(); i++)
  {
    if(angles[i] >= M_PI)
      angles[i] = -2.0*M_PI + angles[i];
  }
}

void EnvironmentCARTROBARM3D::convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action;
  std::vector<double> sangles(ndof_,0), interm_wcoord(ndof_,0), source_wcoord(ndof_,0);
  std::vector<std::vector<double> > interm_angles;
  std::vector<int> interm_coord(ndof_,0);
  EnvROBARM3DHashEntry_t* source_entry;
  std::vector<int>mp_used; 
  path.clear();

  ROS_DEBUG_NAMED(prms_.solution_log_, "[env] Path:");
  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }
    if(bestsucc == -1)
    {
      ROS_ERROR("[env] [%d/%d] Successor not found for transition when attempting to construct the path. (%d -> %d)", int(p), int(idpath.size()), sourceid, targetid);
      return;
    }

    // just added for debugging
    mp_used.push_back(bestsucc);

    source_entry = EnvROBARM.StateID2CoordTable[sourceid];
    sangles = source_entry->angles;
    coordToWorldPose(source_entry->coord, source_wcoord);

    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      if(getJointAnglesForMotionPrimWaypoint(prms_.mp_[bestsucc].m[i], source_wcoord, sangles, interm_wcoord, interm_angles) > 0)
      {
        for(size_t j = 0; j < interm_angles.size(); ++j)
        {
          for(size_t q = 0; q < interm_angles[j].size(); ++q)
            interm_angles[j][q] = angles::normalize_angle(interm_angles[j][q]);

          path.push_back(interm_angles[j]);
        }
        source_wcoord = interm_wcoord;
      }
      else
        ROS_WARN("[env] Failed to convert coords to angles when attempting to construct the path.");
    }
    ROS_DEBUG_NAMED(prms_.solution_log_, "[env] [%2d] stateid: %5d mp_index: %2d mp_type: %14s mp_group: %d num_waypoints: %d  heur: %d", int(p),  idpath[p], bestsucc, prms_.motion_primitive_type_names_[prms_.mp_[bestsucc].type].c_str(), prms_.mp_[bestsucc].group, int(interm_angles.size()), source_entry->heur);  
  }

  for(size_t i = 0; i < mp_used.size(); ++i)
    ROS_INFO("[%d] mp.type: %45s  mp.id: %d", int(i), prms_.motion_primitive_type_names_[prms_.mp_[mp_used[i]].type].c_str(), prms_.mp_[mp_used[i]].id);
}

void EnvironmentCARTROBARM3D::printEnvironmentStats()
{
  stats_.updateAverageChecksPerExpand();
  stats_.printTotalChecksPerSolverSummary();
  stats_.printTotalSolverUsedSummary();
  stats_.printStatsToFile(prms_.environment_type_);
}

void EnvironmentCARTROBARM3D::convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short)
{
  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action, mp_path, idpath2;
  std::vector<double> sangles(ndof_,0), interm_wcoord(ndof_,0), source_wcoord(ndof_,0);
  std::vector<std::vector<double> > interm_angles;
  std::vector<int> interm_coord(ndof_,0);
  EnvROBARM3DHashEntry_t* source_entry;
  path.clear();
  idpath_short.clear();

  if(idpath.size() <= 1)
    return;

  //retrieve list of motion primitives used
  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    printf("[%d] sourceid: %d\n", int(p), sourceid); fflush(stdout);
    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }
    if(bestsucc == -1)
    {
      ROS_ERROR("[env] [%i] Successor not found for transition when attempting to construct the path.", int(p));
      return;
    }
    mp_path.push_back(bestsucc);
  }
  //remove stateids that use the same mprim as previous stateid
  idpath2 = idpath;
  for(int p = int(mp_path.size()-1); p > 0; p--)
  {
    if(mp_path[p] == mp_path[p-1])
      idpath2[p-1] = -1;
  }

  //debugging
  for(size_t p = 0; p < idpath.size()-1; ++p)
    ROS_INFO("[%d] original path: %d  mprim: %d  adjusted path %d", int(p), idpath[p], mp_path[p], idpath2[p]);

  //get joint angles for shortened paths
  for(size_t p = 0; p < idpath2.size()-1; ++p)
  {
    if(idpath2[p] == -1)
      continue;

    idpath_short.push_back(idpath[p]);
    sourceid = idpath2[p];
    source_entry = EnvROBARM.StateID2CoordTable[sourceid];
    sangles = source_entry->angles;
    coordToWorldPose(source_entry->coord, source_wcoord);
    bestsucc = mp_path[p];

    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      if(getJointAnglesForMotionPrimWaypoint(prms_.mp_[bestsucc].m[i], source_wcoord, sangles, interm_wcoord, interm_angles) > 0)
      {
        for(size_t j = 0; j < interm_angles.size(); ++j)
        {
          for(size_t q = 0; q < interm_angles[j].size(); ++q)
            interm_angles[j][q] = angles::normalize_angle(interm_angles[j][q]);

          path.push_back(interm_angles[j]);
        }
        source_wcoord = interm_wcoord;
      }
      else
        ROS_ERROR("[env] Failed to convert coords to angles when attempting to construct the path.");
    }
    ROS_INFO("[env] [%2d] stateid: %5d mp_index: %2d mp_type: %14s mp_group: %d num_waypoints: %d  heur: %d", int(p),  idpath[p], bestsucc, prms_.motion_primitive_type_names_[prms_.mp_[bestsucc].type].c_str(), prms_.mp_[bestsucc].group, int(interm_angles.size()), source_entry->heur); 
  }
}

void EnvironmentCARTROBARM3D::convertStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action;
  std::vector<double> interm_point(4,0), source_wcoord(ndof_,0), wcoord(ndof_,0);
 
  path.clear();

  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }

    if(bestsucc == -1)
      ROS_ERROR("[%i] Successor not found for transition.", int(p));

    coordToWorldPose(EnvROBARM.StateID2CoordTable[sourceid]->coord, source_wcoord);
    
    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      for(size_t a = 0; a < wcoord.size(); ++a)
        wcoord[a] = source_wcoord[a] + prms_.mp_[bestsucc].m[i][a];
      
      ROS_DEBUG("[%i-%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f",int(p),int(i), sourceid, targetid, bestsucc,wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6]);
      
      interm_point[0] = wcoord[0];
      interm_point[1] = wcoord[1];
      interm_point[2] = wcoord[2];
      if(p % 2 == 0)
        interm_point[3] = 0;
      else
        interm_point[3] = 1;

      path.push_back(interm_point);
    }

    ROS_INFO("[%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f  (cell: %d %d %d)",int(p), sourceid, targetid, bestsucc,wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], int(EnvROBARM.StateID2CoordTable[targetid]->coord[0]),int(EnvROBARM.StateID2CoordTable[targetid]->coord[1]),int(EnvROBARM.StateID2CoordTable[targetid]->coord[2]));
  }
}

void EnvironmentCARTROBARM3D::convertShortStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  std::vector<double> interm_point(4,0), wcoord(ndof_,0);
 
  path.clear();

  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    coordToWorldPose(EnvROBARM.StateID2CoordTable[idpath[p]]->coord, wcoord);

    interm_point[0] = wcoord[0];
    interm_point[1] = wcoord[1];
    interm_point[2] = wcoord[2];
    if(p % 2 == 0)
      interm_point[3] = 0;
    else
      interm_point[3] = 1;

    path.push_back(interm_point);
  }
  ROS_INFO("NOTE: convertStateIDPathToPoints will only work with one waypoint motion primitives. It's temporary.");
}

void EnvironmentCARTROBARM3D::getContMotionPrims(char type, std::vector<std::vector<btVector3> > &mprims)
{
  std::vector<btVector3> m;
  btVector3 origin(0.0, 0.0, 0.0);
  mprims.clear();

  for(size_t i = 0; i < prms_.mp_.size(); ++i)
  {
    if(prms_.mp_[i].type == type)
    {
      m.resize(prms_.mp_[i].nsteps+1);
      m[0] = origin;

      for(int j = 0; j < prms_.mp_[i].nsteps; ++j)
      {
        m[j+1].setX(prms_.xyz_resolution_*prms_.mp_[i].m[j][0]);
        m[j+1].setY(prms_.xyz_resolution_*prms_.mp_[i].m[j][1]);
        m[j+1].setZ(prms_.xyz_resolution_*prms_.mp_[i].m[j][2]);

        m[j+1] += m[j];
      }
      mprims.push_back(m);
    }
  }

  ROS_DEBUG("[getContMotionPrimitives] Returning %d motions of type %d.", int(mprims.size()),int(type));
}


bool EnvironmentCARTROBARM3D::isGoalPosition(const std::vector<int> &coord)
{
  if(coord.size() < 6)
    return false;

  if(abs(coord[0]-EnvROBARMCfg.goal.xyz_disc[0]) <= EnvROBARMCfg.goal.xyz_disc_tolerance &&
     abs(coord[1]-EnvROBARMCfg.goal.xyz_disc[1]) <= EnvROBARMCfg.goal.xyz_disc_tolerance &&
     abs(coord[2]-EnvROBARMCfg.goal.xyz_disc[2]) <= EnvROBARMCfg.goal.xyz_disc_tolerance &&
     abs(coord[3]-EnvROBARMCfg.goal.rpy_disc[0]) <= EnvROBARMCfg.goal.rpy_disc_tolerance &&
     abs(coord[4]-EnvROBARMCfg.goal.rpy_disc[1]) <= EnvROBARMCfg.goal.rpy_disc_tolerance &&
     abs(coord[5]-EnvROBARMCfg.goal.rpy_disc[2]) <= EnvROBARMCfg.goal.rpy_disc_tolerance)
  {
    return true;
  }
  return false;
}

void EnvironmentCARTROBARM3D::getAdaptiveMotionPrim(int type, EnvROBARM3DHashEntry_t* parent, MotionPrimitive &mp)
{
  mp.m.clear();
  mp.coord.clear();
  mp.coord.resize(ndof_, 0);

  // set rpy to motion required to satisfy goal orientation
  if(type == SNAP_TO_RPY)
  {
    mp.m.resize(1, std::vector<double> (ndof_, 0.0));
    mp.m[0][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
    mp.m[0][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
    mp.m[0][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);
    mp.coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
    mp.coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
    mp.coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5];
  }
  // snap to goal pose in one motion
  else if(type == SNAP_TO_XYZRPY)
  {
    mp.m.resize(1, std::vector<double> (ndof_, 0.0));
    mp.m[0][0] = double(EnvROBARM.goalHashEntry->coord[0] - parent->coord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[0][1] = double(EnvROBARM.goalHashEntry->coord[1] - parent->coord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[0][2] = double(EnvROBARM.goalHashEntry->coord[2] - parent->coord[2])*EnvROBARMCfg.coord_delta[2];
    mp.m[0][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
    mp.m[0][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
    mp.m[0][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);
    mp.coord[0] = EnvROBARM.goalHashEntry->coord[0] - parent->coord[0];
    mp.coord[1] = EnvROBARM.goalHashEntry->coord[1] - parent->coord[1];
    mp.coord[2] = EnvROBARM.goalHashEntry->coord[2] - parent->coord[2];
    mp.coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
    mp.coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
    mp.coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5];   
  }
  // first satisfy goal orientation in place, then move into the goal position
  else if(type == SNAP_TO_RPY_THEN_TO_XYZ)
  {
    mp.m.resize(2, std::vector<double> (ndof_, 0.0));
    mp.m[0][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
    mp.m[0][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
    mp.m[0][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);
    mp.m[1][0] = double(EnvROBARM.goalHashEntry->coord[0] - parent->coord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[1][1] = double(EnvROBARM.goalHashEntry->coord[1] - parent->coord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[1][2] = double(EnvROBARM.goalHashEntry->coord[2] - parent->coord[2])*EnvROBARMCfg.coord_delta[2];
    mp.coord[0] = EnvROBARM.goalHashEntry->coord[0] - parent->coord[0];
    mp.coord[1] = EnvROBARM.goalHashEntry->coord[1] - parent->coord[1];
    mp.coord[2] = EnvROBARM.goalHashEntry->coord[2] - parent->coord[2];
    mp.coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
    mp.coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
    mp.coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5];
  }
  // first move to the goal position, then rotate into the goal orientation
  else if(type == SNAP_TO_XYZ_THEN_TO_RPY)
  {
    mp.m.resize(2, std::vector<double> (ndof_, 0.0));
    mp.m[0][0] = double(EnvROBARM.goalHashEntry->coord[0] - parent->coord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[0][1] = double(EnvROBARM.goalHashEntry->coord[1] - parent->coord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[0][2] = double(EnvROBARM.goalHashEntry->coord[2] - parent->coord[2])*EnvROBARMCfg.coord_delta[2];
    mp.m[1][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
    mp.m[1][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
    mp.m[1][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);
    mp.coord[0] = EnvROBARM.goalHashEntry->coord[0] - parent->coord[0];
    mp.coord[1] = EnvROBARM.goalHashEntry->coord[1] - parent->coord[1];
    mp.coord[2] = EnvROBARM.goalHashEntry->coord[2] - parent->coord[2];
    mp.coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
    mp.coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
    mp.coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5];
  }
  else if(type == SNAP_TO_RPY_AT_START)
  {
    mp.m.resize(1, std::vector<double> (ndof_, 0.0));
    getVector(EnvROBARM.goalHashEntry->coord[0], EnvROBARM.goalHashEntry->coord[1], EnvROBARM.goalHashEntry->coord[2], parent->coord[0], parent->coord[1], parent->coord[2], mp.coord[0], mp.coord[1], mp.coord[2], 5);  
    getVector(EnvROBARM.goalHashEntry->coord[3], EnvROBARM.goalHashEntry->coord[4], EnvROBARM.goalHashEntry->coord[5], parent->coord[3], parent->coord[4], parent->coord[5], mp.coord[3], mp.coord[4], mp.coord[5], 10); 

    mp.m[0][0] = double(mp.coord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[0][1] = double(mp.coord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[0][2] = double(mp.coord[2])*EnvROBARMCfg.coord_delta[2];
    mp.m[0][3] = angles::normalize_angle(double(mp.coord[3])*EnvROBARMCfg.coord_delta[3]);
    mp.m[0][4] = angles::normalize_angle(double(mp.coord[4])*EnvROBARMCfg.coord_delta[4]);
    mp.m[0][5] = angles::normalize_angle(double(mp.coord[5])*EnvROBARMCfg.coord_delta[5]);
    ROS_DEBUG_NAMED(prms_.expands2_log_, "     [snap_to_rpy_at_start] xyz-coord: %d %d %d  rpy-coord: %d %d %d  fa-coord: %d", mp.coord[0], mp.coord[1], mp.coord[2], mp.coord[3], mp.coord[4], mp.coord[5], mp.coord[6]);
    ROS_DEBUG_NAMED(prms_.expands2_log_, "     [snap_to_rpy_at_start] xyz: %0.3f %0.3f %0.3f    rpy: %0.3f %0.3f %0.3f   fa: %0.3f   (parent-dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6], parent->heur);
  }
  else if(type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ)
  {
    int x,y,z;
    std::vector<int> icoord(ndof_,0);
    if(!getDistanceGradient(x,y,z))
      ROS_ERROR("I shouldn't be here...");

    // get xyz for retracted pose
    getVector(parent->coord[0]+x, parent->coord[1]+y, parent->coord[2]+z, parent->coord[0], parent->coord[1], parent->coord[2], icoord[0], icoord[1], icoord[2], 6, false);
    mp.m.resize(3, std::vector<double> (ndof_, 0.0));
    mp.m[0][0] = double(icoord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[0][1] = double(icoord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[0][2] = double(icoord[2])*EnvROBARMCfg.coord_delta[2];
    mp.m[1][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
    mp.m[1][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
    mp.m[1][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);
    mp.m[2][0] = double(EnvROBARM.goalHashEntry->coord[0] - parent->coord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[2][1] = double(EnvROBARM.goalHashEntry->coord[1] - parent->coord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[2][2] = double(EnvROBARM.goalHashEntry->coord[2] - parent->coord[2])*EnvROBARMCfg.coord_delta[2];
    mp.coord[0] = EnvROBARM.goalHashEntry->coord[0] - parent->coord[0];
    mp.coord[1] = EnvROBARM.goalHashEntry->coord[1] - parent->coord[1];
    mp.coord[2] = EnvROBARM.goalHashEntry->coord[2] - parent->coord[2];
    mp.coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
    mp.coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
    mp.coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5]; 
    

    // debugging
    if(mp.m.size() == 3)
    {
      std::vector<double> wcoord(ndof_,0), gwcoord(ndof_,0);
      coordToWorldPose(parent->coord, wcoord);
      coordToWorldPose(EnvROBARM.goalHashEntry->coord, gwcoord);
      ROS_DEBUG_NAMED(prms_.expands_log_, " [distance_gradiant] %d %d %d", x,y,z);
      ROS_DEBUG_NAMED(prms_.expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",gwcoord[0]-wcoord[0],gwcoord[1]-wcoord[1],gwcoord[2]-wcoord[2],gwcoord[3]-wcoord[3],gwcoord[4]-wcoord[4],gwcoord[5]-wcoord[5],gwcoord[6]-wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[1][0],mp.m[1][1],mp.m[1][2],mp.m[1][3],mp.m[1][4],mp.m[1][5],mp.m[1][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[2][0],mp.m[2][1],mp.m[2][2],mp.m[2][3],mp.m[2][4],mp.m[2][5],mp.m[2][6], parent->heur);
    }
    else
    {
      std::vector<double> wcoord(ndof_,0), gwcoord(ndof_,0);
      coordToWorldPose(parent->coord, wcoord);
      coordToWorldPose(EnvROBARM.goalHashEntry->coord, gwcoord);
      ROS_DEBUG_NAMED(prms_.expands_log_, " [distance_gradiant] %d %d %d", x,y,z);
      ROS_DEBUG_NAMED(prms_.expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",gwcoord[0]-wcoord[0],gwcoord[1]-wcoord[1],gwcoord[2]-wcoord[2],gwcoord[3]-wcoord[3],gwcoord[4]-wcoord[4],gwcoord[5]-wcoord[5],gwcoord[6]-wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-rpy]  -- ");
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[1][0],mp.m[1][1],mp.m[1][2],mp.m[1][3],mp.m[1][4],mp.m[1][5],mp.m[1][6], parent->heur);
    }
  }
  else if(type == RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ)
  {
    int x,y,z;
    std::vector<int> icoord(ndof_,0);
    if(!getDistanceGradient(x,y,z))
      ROS_ERROR("I shouldn't be here...");

    // get xyz for retracted pose
    getVector(parent->coord[0]+x, parent->coord[1]+y, parent->coord[2]+z, parent->coord[0], parent->coord[1], parent->coord[2], icoord[0], icoord[1], icoord[2], 4, false);

    // rotate towards rpy
    getVector(EnvROBARM.goalHashEntry->coord[3], EnvROBARM.goalHashEntry->coord[4], EnvROBARM.goalHashEntry->coord[5], parent->coord[3], parent->coord[4], parent->coord[5], mp.coord[3], mp.coord[4], mp.coord[5], 15); 

    std::vector<double> im(ndof_, 0); 
    mp.m.resize(1, std::vector<double> (ndof_, 0.0));
    mp.m[0][0] = double(icoord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[0][1] = double(icoord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[0][2] = double(icoord[2])*EnvROBARMCfg.coord_delta[2];
    
    if(mp.coord[3] != 0 || mp.coord[4] != 0 || mp.coord[5] != 0)
    {
      im[3] = angles::normalize_angle(double(mp.coord[3])*EnvROBARMCfg.coord_delta[3]);
      im[4] = angles::normalize_angle(double(mp.coord[4])*EnvROBARMCfg.coord_delta[4]);
      im[5] = angles::normalize_angle(double(mp.coord[5])*EnvROBARMCfg.coord_delta[5]);
      mp.m.push_back(im);
    }
      std::vector<double> wcoord(ndof_,0), gwcoord(ndof_,0);
      coordToWorldPose(parent->coord, wcoord);
      coordToWorldPose(EnvROBARM.goalHashEntry->coord, gwcoord);
      ROS_DEBUG_NAMED(prms_.expands_log_, " [distance_gradiant] %d %d %d", x,y,z);
      ROS_DEBUG_NAMED(prms_.expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",gwcoord[0]-wcoord[0],gwcoord[1]-wcoord[1],gwcoord[2]-wcoord[2],gwcoord[3]-wcoord[3],gwcoord[4]-wcoord[4],gwcoord[5]-wcoord[5],gwcoord[6]-wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[1][0],mp.m[1][1],mp.m[1][2],mp.m[1][3],mp.m[1][4],mp.m[1][5],mp.m[1][6], parent->heur);

    /*
    int x,y,z;
    std::vector<int> icoord(ndof_,0);
    if(!getDistanceGradient(x,y,z))
      ROS_ERROR("I shouldn't be here...");

    // get xyz for retracted pose
    getVector(parent->coord[0]+x, parent->coord[1]+y, parent->coord[2]+z, parent->coord[0], parent->coord[1], parent->coord[2], icoord[0], icoord[1], icoord[2], 4, false);

    // rotate towards rpy
    getVector(EnvROBARM.goalHashEntry->coord[3], EnvROBARM.goalHashEntry->coord[4], EnvROBARM.goalHashEntry->coord[5], parent->coord[3], parent->coord[4], parent->coord[5], mp.coord[3], mp.coord[4], mp.coord[5], 10); 

    // move towards xyz
    getVector(EnvROBARM.goalHashEntry->coord[0], EnvROBARM.goalHashEntry->coord[1], EnvROBARM.goalHashEntry->coord[2], parent->coord[0], parent->coord[1], parent->coord[2], mp.coord[0], mp.coord[1], mp.coord[2], 6);

    std::vector<double> im(ndof_, 0); 
    mp.m.resize(1, std::vector<double> (ndof_, 0.0));
    mp.m[0][0] = double(icoord[0])*EnvROBARMCfg.coord_delta[0];
    mp.m[0][1] = double(icoord[1])*EnvROBARMCfg.coord_delta[1];
    mp.m[0][2] = double(icoord[2])*EnvROBARMCfg.coord_delta[2];
    
    if(mp.coord[3] != 0 || mp.coord[4] != 0 || mp.coord[5] != 0)
    {
      im[3] = angles::normalize_angle(double(mp.coord[3])*EnvROBARMCfg.coord_delta[3]);
      im[4] = angles::normalize_angle(double(mp.coord[4])*EnvROBARMCfg.coord_delta[4]);
      im[5] = angles::normalize_angle(double(mp.coord[5])*EnvROBARMCfg.coord_delta[5]);
      mp.m.push_back(im);
    }
    
    im[0] = double(mp.coord[0])*EnvROBARMCfg.coord_delta[0];
    im[1] = double(mp.coord[1])*EnvROBARMCfg.coord_delta[1];
    im[2] = double(mp.coord[2])*EnvROBARMCfg.coord_delta[2];
    im[3] = 0;
    im[4] = 0;
    im[5] = 0;
    mp.m.push_back(im);

    // debugging
    if(mp.m.size() == 3)
    {
      std::vector<double> wcoord(ndof_,0), gwcoord(ndof_,0);
      coordToWorldPose(parent->coord, wcoord);
      coordToWorldPose(EnvROBARM.goalHashEntry->coord, gwcoord);
      ROS_DEBUG_NAMED(prms_.expands_log_, " [distance_gradiant] %d %d %d", x,y,z);
      ROS_DEBUG_NAMED(prms_.expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",gwcoord[0]-wcoord[0],gwcoord[1]-wcoord[1],gwcoord[2]-wcoord[2],gwcoord[3]-wcoord[3],gwcoord[4]-wcoord[4],gwcoord[5]-wcoord[5],gwcoord[6]-wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[1][0],mp.m[1][1],mp.m[1][2],mp.m[1][3],mp.m[1][4],mp.m[1][5],mp.m[1][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[2][0],mp.m[2][1],mp.m[2][2],mp.m[2][3],mp.m[2][4],mp.m[2][5],mp.m[2][6], parent->heur);
    }
    else
    {
      std::vector<double> wcoord(ndof_,0), gwcoord(ndof_,0);
      coordToWorldPose(parent->coord, wcoord);
      coordToWorldPose(EnvROBARM.goalHashEntry->coord, gwcoord);
      ROS_DEBUG_NAMED(prms_.expands_log_, " [distance_gradiant] %d %d %d", x,y,z);
      ROS_DEBUG_NAMED(prms_.expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",gwcoord[0]-wcoord[0],gwcoord[1]-wcoord[1],gwcoord[2]-wcoord[2],gwcoord[3]-wcoord[3],gwcoord[4]-wcoord[4],gwcoord[5]-wcoord[5],gwcoord[6]-wcoord[6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6], parent->heur);
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-rpy]  -- ");
      ROS_DEBUG_NAMED(prms_.expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)",mp.m[1][0],mp.m[1][1],mp.m[1][2],mp.m[1][3],mp.m[1][4],mp.m[1][5],mp.m[1][6], parent->heur);
    }
    */
  }
  else
    ROS_WARN("[env] Invalid adaptive motion primitive type.");
}

int EnvironmentCARTROBARM3D::getJointAnglesForMotionPrimWaypoint(const std::vector<double> &mp_point, const std::vector<double> &init_wcoord, const std::vector<double> &pangles, std::vector<double> &final_wcoord, std::vector<std::vector<double> > &angles)
{
  int status = 0;
  std::vector<double> pose(6,0);
  std::vector<double> seed = pangles;

  for(size_t a = 0; a < 6; ++a)
    pose[a] = init_wcoord[a] + mp_point[a];

  seed[free_angle_index_] = init_wcoord[6] + mp_point[6];
  final_wcoord = pose;
  final_wcoord.push_back(seed[free_angle_index_]);

  // orientation solver - for rpy motion
  if(mp_point[0] == 0 && mp_point[1] == 0 && mp_point[2] == 0 && mp_point[6] == 0)
  {
    if(!rpysolver_->isOrientationFeasible(EnvROBARMCfg.goal.rpy, seed, angles))
      status = -solver_types::ORIENTATION_SOLVER;
    else
      return solver_types::ORIENTATION_SOLVER; 
  }

  // analytical IK
  angles.resize(1, std::vector<double> (ndof_,0));
  if(!kmodel_->computeFastIK(pose,seed,angles[0]))
  {
    status = -solver_types::IK;
    
    // IK search
    if(!kmodel_->computeIK(pose, seed, angles[0]))
      status = -solver_types::IK_SEARCH;
    else
    {
      final_wcoord[6] = angles[0][free_angle_index_];
      return solver_types::IK_SEARCH;
    }
  }
  else
    return solver_types::IK;

  return status;
}

/*
int EnvironmentCARTROBARM3D::isMotionValid(const std::vector<double> &start, const std::vector<double> &end, int &motion_length, int &nchecks, unsigned char &dist)
{
  //check joint limits of end configuration (assume start is valid)
  if(!arm_->checkJointLimits(end))
    return -1;

  //check for collisions
  nchecks = 1;
  if(!cc_->checkCollision(end, prms_.verbose_, false, dist))
    return -2;

  // check for collision along interpolated path
  if(!cspace_->checkPathForCollision(start, end, prms_.verbose_, motion_length, nchecks, dist))
  {
    nchecks++; // to include the one from the call to checkCollision above
    return -3;
  }
  return 1;
}
*/

bool EnvironmentCARTROBARM3D::getMotionPrimitive(EnvROBARM3DHashEntry_t* parent, MotionPrimitive &mp)
{
  if(mp.type == LONG_DISTANCE)
  {
    if(parent->heur <= prms_.short_dist_mprims_thresh_c_ && prms_.use_multires_mprims_)
      return false;
  }
  else if(mp.type == SHORT_DISTANCE)
  {
    if(parent->heur > prms_.short_dist_mprims_thresh_c_ && prms_.use_multires_mprims_)
      return false;
  }
  else if(mp.type == SNAP_TO_RPY)
  {
    if(parent->heur > prms_.cost_per_cell_ * 6)
      return false;
    getAdaptiveMotionPrim(SNAP_TO_RPY, parent, mp);
  }
  else if(mp.type == SNAP_TO_XYZRPY)
  {
    if(parent->heur > prms_.cost_per_cell_ * 10)
      return false;
    getAdaptiveMotionPrim(SNAP_TO_XYZRPY, parent, mp);
  }
  else if(mp.type == SNAP_TO_XYZ_THEN_TO_RPY)
  {
    if(parent->heur > prms_.cost_per_cell_ * 15)
      return false;
    getAdaptiveMotionPrim(SNAP_TO_XYZ_THEN_TO_RPY, parent, mp);
  }
  else if(mp.type == SNAP_TO_RPY_THEN_TO_XYZ)
  {
    if(parent->heur > prms_.cost_per_cell_ * 15)
      return false;
    getAdaptiveMotionPrim(SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
  }
  else if(mp.type == SNAP_TO_RPY_AT_START)
  {
    if(parent->heur < prms_.cost_per_cell_ * 3)
      return false;
    getAdaptiveMotionPrim(SNAP_TO_RPY_AT_START, parent, mp);
  }
  else if(mp.type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ)
  {
    if(parent->heur > prms_.cost_per_cell_ * 20)
      return false;
    int x, y, z;
    if(!getDistanceGradient(x,y,z))
      return false;
    getAdaptiveMotionPrim(RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
  }
  else if(mp.type == RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ)
  {
    if(EnvROBARM.goalHashEntry->coord[3] == parent->coord[3] &&
        EnvROBARM.goalHashEntry->coord[4] == parent->coord[4] &&
        EnvROBARM.goalHashEntry->coord[5] == parent->coord[5])
    {
        ROS_DEBUG_NAMED(prms_.expands2_log_, "Already at goal rpy. Not doing the retract motion.");
        return false;
    }

    int x,y,z;
    if(!getDistanceGradient(x,y,z))
    {
      ROS_ERROR("Zero GRadient");
      return false;
    }


    /*
    if(parent->heur > prms_.cost_per_cell_ * 40)
      return false;
    int x, y, z;
    if(!getDistanceGradient(x,y,z))
      return false;
    if(parent->heur > prms_.cost_per_cell_ * 5)
      getAdaptiveMotionPrim(RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ, parent, mp);
    else
      getAdaptiveMotionPrim(RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
    */
      
    getAdaptiveMotionPrim(RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ, parent, mp);
  }
  else
    ROS_WARN("Invalid motion primitive type");

  if(mp.coord[0] == 0 && mp.coord[1] == 0 && mp.coord[2] == 0 &&
      mp.coord[3] == 0 && mp.coord[4] == 0 && mp.coord[5] == 0 && mp.coord[6] == 0) 
  {
    ROS_DEBUG("Not using adaptive mprim cause its all zeros. We should be at goal??? (type: %s)", prms_.motion_primitive_type_names_[mp.type].c_str());
    return false;
  }

  /*
  if(mp.type > ADAPTIVE)
    ROS_INFO("   adaptive-mp: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f   rpy-coord: %d %d %d (dist: %d)",mp.m[0][0],mp.m[0][1],mp.m[0][2],mp.m[0][3],mp.m[0][4],mp.m[0][5],mp.m[0][6],  mp.coord[3], mp.coord[4], mp.coord[5], parent->heur);
  */

  return true;
}

void EnvironmentCARTROBARM3D::getVector(int x1, int y1, int z1, int x2, int y2, int z2, int &xout, int &yout, int &zout, int multiplier, bool snap)
{
  double dx = x1 - x2;
  double dy = y1 - y2;
  double dz = z1 - z2;
  double length = sqrt(dx*dx + dy*dy + dz*dz);

  if(!snap)
  {
    if(dx != 0)
      xout = (dx*multiplier)/length + 0.5;
    else
      xout = 0;

    if(dy != 0)
      yout = (dy*multiplier)/length + 0.5;
    else
      yout = 0;

    if(dz != 0)
      zout = (dz*multiplier)/length + 0.5;
    else 
      zout = 0;
  }
  else
  {
    if(dx > 0)
      xout = min((dx*multiplier)/length + 0.5, dx);
    else if(dx < 0)
      xout = max((dx*multiplier)/length + 0.5, dx);
    else
      xout = 0;

    if(dy > 0)
      yout = min((dy*multiplier)/length + 0.5, dy);
    else if(dy < 0)
      yout = max((dy*multiplier)/length + 0.5, dy);
    else
      yout = 0;

    if(dz > 0)
      zout = min((dz*multiplier)/length + 0.5, dz);
    else if(dz < 0)
      zout = max((dz*multiplier)/length + 0.5, dz);
    else 
      zout = 0;
  }
  ROS_DEBUG("xyz1: %d %d %d    xyz2: %d %d %d", x1, y1, z1, x2, y2, z2);
  ROS_DEBUG("dx: %1.2f dy: %1.2f dz: %1.2f length: %1.2f multiplier: %d unit_double{%1.2f %1.2f %1.2f}  unit_int{%d %d %d}", dx, dy, dz, length, multiplier, dx/length, dy/length, dz/length, xout, yout, zout);
}

bool EnvironmentCARTROBARM3D::getDistanceGradient(int &x, int &y, int &z)
{
  mp_gradient_[0] = mp_dist_[0] - mp_dist_[1];
  mp_gradient_[1] = mp_dist_[2] - mp_dist_[3];
  mp_gradient_[2] = mp_dist_[4] - mp_dist_[5];

  x = mp_gradient_[0];
  y = mp_gradient_[1];
  z = mp_gradient_[2];

  if(mp_dist_[0] == -1 ||  mp_dist_[1] == -1)
  {
    mp_gradient_[0] = 0;
    return false;
  }
  if(mp_dist_[2] == -1 ||  mp_dist_[3] == -1)
  {
    mp_gradient_[1] = 0;
    return false;
  }
  if(mp_dist_[4] == -1 ||  mp_dist_[5] == -1)
  {
    mp_gradient_[2] = 0;
    return false;
  }
  if(mp_gradient_[0] == 0 && mp_gradient_[1] == 0 && mp_gradient_[2] == 0)
    return false;

  //ROS_INFO("[env] dist:  %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f  (%s)", mp_dist_[0], mp_dist_[1], mp_dist_[2], mp_dist_[3], mp_dist_[4], mp_dist_[5], cspace_->collision_name_.c_str());      
  double norm = sqrt(mp_gradient_[0]*mp_gradient_[0] + mp_gradient_[1]*mp_gradient_[1] + mp_gradient_[2]*mp_gradient_[2]);
  ROS_INFO("[env] gradient_x: %2.2f   gradient_y: %2.2f   gradient_z: %2.2f  norm: %2.2f", mp_gradient_[0], mp_gradient_[1], mp_gradient_[2], norm);
  return true;
}

void EnvironmentCARTROBARM3D::computeGradient(const MotionPrimitive &mp, unsigned char &d, bool collision)
{
  if(mp.id < 6 && mp.group == 1)
  {
    if(d == 100)
      mp_dist_[mp.id] = -1;
    else 
      mp_dist_[mp.id] = d;
    /*
    if(d == 0)
      ROS_ERROR("%d: distance gradient = 0", mp.id);
    aviz_->deleteVisualizations("mp_gradient", 0);
    int hue = 160;
    if(collision)
    {
      hue = 30;
    }
      //aviz_->visualizeSphere(cspace_->collision_, hue, "collision_sphere", cspace_->collision_[3]);
      aviz_->visualizeSphere(cspace_->collision_, hue, "mp_gradient_"+boost::lexical_cast<std::string>(mp.id), cspace_->collision_[3]);
      geometry_msgs::Pose pose;
      pose.position.x = cspace_->collision_[0];
      pose.position.y = cspace_->collision_[1];
      pose.position.z = cspace_->collision_[2];
      std::vector<double> color(4,1);
      aviz_->visualizeText(pose, cspace_->collision_name_, "mp_gradient_"+boost::lexical_cast<std::string>(mp.id), 3, color, 0.02);
      //aviz_->visualizeText(pose, cspace_->collision_name_, "collision_sphere", 3, color, 0.02);
    //}
    //ROS_INFO("[env] %s %d", cspace_->collision_name_.c_str(), int(d)); 
    */
  }
}

int EnvironmentCARTROBARM3D::getXYZRPYHeuristic(int FromStateID, int ToStateID)
{
  int xyz_heur = 0, rpy_heur = 0;
  EnvROBARM3DHashEntry_t* state = EnvROBARM.StateID2CoordTable[FromStateID];

  xyz_heur = getBFSCostToGoal(state->coord[0], state->coord[1], state->coord[2]);

  int r = (EnvROBARM.goalHashEntry->coord[3] - state->coord[3]) % EnvROBARMCfg.coord_vals[3];
  int p = (EnvROBARM.goalHashEntry->coord[4] - state->coord[4]) % EnvROBARMCfg.coord_vals[4];
  int y = (EnvROBARM.goalHashEntry->coord[5] - state->coord[5]) % EnvROBARMCfg.coord_vals[5];

  rpy_heur = (abs(r)+abs(p)+abs(y)) * prms_.cost_per_cell_;
  ROS_DEBUG("stateid: %5d   xyz: %2d %2d %2d  rpy: %2d %2d %2d  (goal-rpy: %2d %2d %2d)", FromStateID, state->coord[0], state->coord[1], state->coord[2],state->coord[3], state->coord[4], state->coord[5], EnvROBARM.goalHashEntry->coord[3],EnvROBARM.goalHashEntry->coord[4],EnvROBARM.goalHashEntry->coord[5]);
  ROS_DEBUG("               xyz-heur: %4d  rpy-heur: %4d  (r: %2d  p:%2d  y: %2d)", xyz_heur, rpy_heur, r, p, y);
  
  state->heur = xyz_heur + rpy_heur;
  return xyz_heur + rpy_heur;
}

}

