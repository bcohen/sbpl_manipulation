/*
 * Copyright (c) 2008, Maxim Likhachev
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

#ifndef _ENVIRONMENT_ROBARM3D_H_
#define _ENVIRONMENT_ROBARM3D_H_

#include <time.h>
#include <vector>
#include <string>
#include <angles/angles.h>
#include <bfs3d/BFS_3D.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_manipulation_components/robot_model.h>
#include <sbpl_manipulation_components/collision_checker.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/planning_params.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace sbpl_arm_planner {

enum GoalType
{
  XYZ_GOAL,
  XYZ_RPY_GOAL,
  NUMBER_OF_GOAL_TYPES
};

typedef struct
{
  int type;
  std::vector<double> pose;
  double xyz_tolerance[3];
  double rpy_tolerance[3];
} GoalConstraint;

/** @brief struct that describes a basic pose constraint */
/*
typedef struct
{
  bool is_6dof_goal;
  int type;
  int xyz_disc_tolerance;
  int rpy_disc_tolerance;
  int xyz_disc[3];
  int rpy_disc[3];
  double xyz[3];
  double rpy[3];
  double q[4];
  double fangle;
  double xyz_tolerance[3];
  double rpy_tolerance[3];
} GoalPos;
*/

typedef struct
{
  int stateID;             // hash entry ID number
  int heur;
  int xyz[3];              // planning link pos (xyz)
  double dist;
  std::vector<int> coord;
  RobotState state;
} EnvROBARM3DHashEntry_t;

/** main structure that stores environment data used in planning */
typedef struct EnvironmentPlanningData
{
  bool near_goal;
  clock_t t_start;
  double time_to_goal_region;
  GoalConstraint goal;

  EnvROBARM3DHashEntry_t* goal_entry;
  EnvROBARM3DHashEntry_t* start_entry;

  // maps from coords to stateID
  int HashTableSize;
  std::vector<EnvROBARM3DHashEntry_t*>* Coord2StateIDHashTable;

  // maps from stateID to coords	
  std::vector<EnvROBARM3DHashEntry_t*> StateID2CoordTable;

  // stateIDs of expanded states
  std::vector<int> expanded_states;

  EnvironmentPlanningData()
  {
    near_goal = false;
    start_entry = NULL;
    goal_entry = NULL;
    Coord2StateIDHashTable = NULL;
  }

  void init()
  {
    HashTableSize = 32*1024; //should be power of two
    Coord2StateIDHashTable = new std::vector<EnvROBARM3DHashEntry_t*>[HashTableSize];
    StateID2CoordTable.clear();
  }
} EnvironmentPlanningData;


/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentROBARM3D: public DiscreteSpaceInformation 
{
  public:

    EnvironmentROBARM3D(OccupancyGrid *grid, RobotModel *rmodel, CollisionChecker *cc, ActionSet* as, PlanningParams *pm);
    ~EnvironmentROBARM3D();

    virtual bool AreEquivalent(int StateID1, int StateID2);
    virtual bool setStartConfiguration(std::vector<double> angles);
    virtual bool setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances);
    virtual void getExpandedStates(std::vector<std::vector<double> >* ara_states);
    virtual void convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path){};
    virtual bool convertStateIDPathToJointTrajectory(const std::vector<int> &idpath, trajectory_msgs::JointTrajectory &traj);
    virtual void convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short){};
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    virtual void StateID2Angles(int stateID, std::vector<double> &angles);
    virtual int getXYZRPYHeuristic(int FromStateID, int ToStateID){return 0;};

    bool initEnvironment();
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    bool InitializeEnv(const char* sEnvFile){return false;};
    int GetFromToHeuristic(int FromStateID, int ToStateID);
    int GetGoalHeuristic(int stateID);
    int GetStartHeuristic(int stateID);
    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    int	SizeofCreatedEnv();
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    void SetAllPreds(CMDPSTATE* state);
    void PrintEnv_Config(FILE* fOut);
    
    RobotModel* getRobotModel(){ return rmodel_; };
    CollisionChecker* getCollisionChecker(){ return cc_; };
    std::vector<double> getGoal();
    double getDistanceToGoal(double x, double y, double z);

    visualization_msgs::MarkerArray getVisualization(std::string type);

  protected:

    OccupancyGrid *grid_;
    RobotModel *rmodel_;
    CollisionChecker *cc_;
    BFS_3D *bfs_;
    ActionSet *as_;

    EnvironmentPlanningData pdata_;
    PlanningParams *prm_;

    // function pointers for heuristic function
    int (EnvironmentROBARM3D::*getHeuristic_) (int FromStateID, int ToStateID);

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<int> &coord);
    virtual EnvROBARM3DHashEntry_t* getHashEntry(const std::vector<int> &coord, bool bIsGoal);
    virtual EnvROBARM3DHashEntry_t* createHashEntry(const std::vector<int> &coord, int endeff[3]);

    /** coordinate frame/angle functions */
    virtual void coordToAngles(const std::vector<int> &coord, std::vector<double> &angles);
    virtual void anglesToCoord(const std::vector<double> &angle, std::vector<int> &coord);

    /** planning */
    virtual bool isGoalState(const std::vector<double> &pose, GoalConstraint &goal);

    /** costs */
    int cost(EnvROBARM3DHashEntry_t* HashEntry1, EnvROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal);
    int getEdgeCost(int FromStateID, int ToStateID);
    virtual void computeCostPerCell();
    int getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist);

    /** output */
    void printHashTableHist();
    void printJointArray(FILE* fOut, EnvROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose);

    /** distance */
    int getBFSCostToGoal(int x, int y, int z) const;
    virtual int getXYZHeuristic(int FromStateID, int ToStateID);
    double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) const;
};


inline unsigned int EnvironmentROBARM3D::intHash(unsigned int key)
{
  key += (key << 12); 
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

inline unsigned int EnvironmentROBARM3D::getHashBin(const std::vector<int> &coord)
{
  int val = 0;

  for(size_t i = 0; i < coord.size(); i++)
    val += intHash(coord[i]) << i;

  return intHash(val) & (pdata_.HashTableSize-1);
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentROBARM3D::coordToAngles(const std::vector<int> &coord, std::vector<double> &angles)
{
  angles.resize(coord.size());
  for(size_t i = 0; i < coord.size(); i++)
    angles[i] = coord[i] * prm_->coord_delta_[i];
}

inline void EnvironmentROBARM3D::anglesToCoord(const std::vector<double> &angle, std::vector<int> &coord)
{
  double pos_angle;

  for(int i = 0; i < int(angle.size()); i++)
  {
    pos_angle = angle[i];
    if(pos_angle < 0.0)
      pos_angle += 2*M_PI;

    coord[i] = (int)((pos_angle + prm_->coord_delta_[i]*0.5)/prm_->coord_delta_[i]);

    if(coord[i] == prm_->coord_vals_[i])
      coord[i] = 0;
  }
}

inline double EnvironmentROBARM3D::getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) const
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

} //namespace

#endif

