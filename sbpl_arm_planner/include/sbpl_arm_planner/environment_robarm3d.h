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
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <string>
#include <list>
#include <algorithm>
#include <angles/angles.h>
#include <bfs3d/BFS_3D.h>
#include <sbpl/headers.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_manipulation_components/robot_model.h>
#include <sbpl_manipulation_components/collision_checker.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/sbpl_arm_planner_params.h>

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
} GoalState;

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

/* @brief an outdated struct that is gradually being torn apart */
typedef struct
{
  bool bInitialized;
  //GoalPos goal;
  std::vector<double> coord_delta;
  std::vector<int> coord_vals;
} EnvROBARM3DConfig_t;

/** main structure that stores environment data used in planning */
typedef struct
{
  EnvROBARM3DHashEntry_t* goalHashEntry;
  EnvROBARM3DHashEntry_t* startHashEntry;

  //maps from coords to stateID
  int HashTableSize;
  std::vector<EnvROBARM3DHashEntry_t*>* Coord2StateIDHashTable;

  //vector that maps from stateID to coords	
  std::vector<EnvROBARM3DHashEntry_t*> StateID2CoordTable;

} EnvironmentROBARM3D_t;


/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentROBARM3D: public DiscreteSpaceInformation 
{
  public:

    std::vector<int> expanded_states;
    bool save_expanded_states;

    /**
     * @brief Default constructor
    */
    EnvironmentROBARM3D(OccupancyGrid *grid, RobotModel *rmodel, CollisionChecker *cc, ActionSet* as);

    /**
     * @brief Destructor
    */
    ~EnvironmentROBARM3D();

    /** 
     * @brief Initialize the environment from a text file 
     * @param name of environment text file
     * @return true if successful, false otherwise
    */
    virtual bool InitializeEnv(const char* sEnvFile);
    
    /**
     * @brief Check if the states with StateID1 & StateID2 are equivalent
     * based on an equivalency function with some declared tolerance.
     * @param stateID of first state
     * @param stateID of second state
     * @return true if equivalent, false otherwise
    */
    virtual bool AreEquivalent(int StateID1, int StateID2);

    /*!
     * @brief Initialize the environment and arm planner parameters from
     * text files. Also, initialize KDL chain from a URDF file.
     * @param is pointer to file describing the environment
     * @param is a pointer to file with the Arm planner parameters
     * @param is a URDF describing the manipulator
     * @return true if successful, false otherwise
    */
    bool initEnvironment(std::string mprims_filename, std::string urdf, std::string srdf);

    /**
     * @brief Initialize the start and goal states of the MDP
     * @param always returns true...
    */
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    /**
     * @brief Set the initial joint configuration of the manipulator. Needs
     * to be set every time the planner is called.
     * @param an array of joint angles
     * @return true if successful, false otherwise
    */
    virtual bool setStartConfiguration(std::vector<double> angles);

    /**
     * @brief Get the heuristic from one state to another state. 
     * @param the stateID of the current state
     * @param the stateID of the goal state
     * @return h(s,s')
    */
    int GetFromToHeuristic(int FromStateID, int ToStateID);
    
    /**
     * @brief Get the heuristic of a state to the planner's goal state.
     * @param the stateID of the current state
     * @return h(s,s_goal)
    */
    int GetGoalHeuristic(int stateID);

    /**
     * @brief Get the heuristic of a state to the planner's start state.
     * @param the stateID of the current state
     * @return h(s,s_start)
    */
    int GetStartHeuristic(int stateID);

    /** 
     * @brief Get the successors of the desired state to be expanded.
     * Return vectors with the successors' state IDs and the cost to move
     * from the current state to that state. If the vectors return to the
     * planner empty then the search quits.
     * @param the state ID of the state to be expanded
     * @param a pointer to a vector that will be populated with the
     * successor state IDs.
     * @param a pointer to a vector that will be populated with the costs of
     * transitioning from the current state to the successor state.
    */
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    
    /** @brief Not defined. */
    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    
    /** 
     * @brief This function searches the hash table for a state by
     * a stateID and returns the joint angles of that entry.
     * @param the stateID of the state to fetch
     * @param a vector of joint angles 
    */
    virtual void StateID2Angles(int stateID, std::vector<double> &angles);

    /** 
     * @brief This function returns the number of hash entries created.
     * @return number of hash entries
    */
    int	SizeofCreatedEnv();

    /**
     * @brief This function prints out the state information of a state.
     * @param the state ID of the desired state
     * @param prints out a little extra information
     * @param the file pointer to print to (stdout by default)
    */
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

    /** @brief Not defined. */
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /** @brief Not defined. */
    void SetAllPreds(CMDPSTATE* state);

    /** @brief Not defined. */
    void PrintEnv_Config(FILE* fOut);

    /**
     * @brief This function sets the goal position. It must be called before
     * starting a new search.
     * @param a 2D vector of pose constraints {{x1,y1,z1,r1,p1,y1},...}
     * @param a 2D vector of tolerances on the pose constraints
     * {{allowed_err_meters1,allowed_error_radians1},...}
     * @return true if succesful, false otherwise
    */
    virtual bool setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances);

    /**
     * @brief This function is for debugging purposes. It returns the
     * pose of the states that were expanded. The planner node has
     * a function to display these as visualizations in rviz.
     * @param a pointer to a vector of the poses of all of the states
     * expanded during the search (when using ARA*)
    */
    virtual void getExpandedStates(std::vector<std::vector<double> >* ara_states);

    /* Cartesian Arm Planner only */
    virtual void convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path){};

    virtual void convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short){};
    
    virtual int getXYZRPYHeuristic(int FromStateID, int ToStateID){return 0;};

    RobotModel* getRobotModel(){return rmodel_;};
    
    std::vector<double> getGoal();

    double getDistanceToGoal(double x, double y, double z);

  protected:

    EnvROBARM3DConfig_t EnvROBARMCfg;
    EnvironmentROBARM3D_t EnvROBARM;

    OccupancyGrid *grid_;
    RobotModel *rmodel_;
    CollisionChecker *cc_;
    SBPLArmPlannerParams prms_;
    BFS_3D *bfs_;
    ActionSet *as_;

    //USED TO BE STATIC VARS
    bool near_goal;
    clock_t starttime;
    double time_to_goal_region;

    // NEW THINGS WE ARE ADDING
    int num_joints_;
    int heuristic_sphere_;
    std::vector<std::string> planning_joints_;

    GoalState goal_;

    // function pointers for heuristic function
    int (EnvironmentROBARM3D::*getHeuristic_) (int FromStateID, int ToStateID);

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<int> &coord);
    virtual EnvROBARM3DHashEntry_t* getHashEntry(const std::vector<int> &coord, int action, bool bIsGoal);
    virtual EnvROBARM3DHashEntry_t* createHashEntry(const std::vector<int> &coord, int endeff[3]);

    /** initialization */
    virtual bool initEnvConfig();
    void initHeuristics();

    /** coordinate frame/angle functions */
    virtual void discretizeAngles();
    virtual void coordToAngles(const std::vector<int> &coord, std::vector<double> &angles);
    virtual void anglesToCoord(const std::vector<double> &angle, std::vector<int> &coord);

    /** planning */
    virtual bool isGoalState(const std::vector<double> &pose, GoalState &goal);

    /** costs */
    int cost(EnvROBARM3DHashEntry_t* HashEntry1, EnvROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal);
    int getEdgeCost(int FromStateID, int ToStateID);
    virtual void computeCostPerCell();
    void computeCostPerRadian();
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

  return intHash(val) & (EnvROBARM.HashTableSize-1);
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentROBARM3D::coordToAngles(const std::vector<int> &coord, std::vector<double> &angles)
{
  for(size_t i = 0; i < coord.size(); i++)
    angles[i] = coord[i]*EnvROBARMCfg.coord_delta[i];
}

inline void EnvironmentROBARM3D::anglesToCoord(const std::vector<double> &angle, std::vector<int> &coord)
{
  double pos_angle;

  for(int i = 0; i < int(angle.size()); i++)
  {
    pos_angle = angle[i];
    if(pos_angle < 0.0)
      pos_angle += 2*M_PI;

    coord[i] = (int)((pos_angle + EnvROBARMCfg.coord_delta[i]*0.5)/EnvROBARMCfg.coord_delta[i]);

    if(coord[i] == EnvROBARMCfg.coord_vals[i])
      coord[i] = 0;
  }
}

inline double EnvironmentROBARM3D::getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) const
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

} //namespace

#endif

