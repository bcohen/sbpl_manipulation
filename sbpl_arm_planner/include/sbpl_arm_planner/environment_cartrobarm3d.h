/*
 * Copyright (c) 2011, Maxim Likhachev
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

/** \Author: Benjamin Cohen /bcohen@seas.upenn.edu **/

#ifndef _ENVIRONMENT_CARTROBARM3D_H_
#define _ENVIRONMENT_CARTROBARM3D_H_

#include <LinearMath/btVector3.h>
#include <sbpl_arm_planner/environment_robarm3d.h>

namespace sbpl_arm_planner {

class EnvironmentCARTROBARM3D: public EnvironmentROBARM3D
{
  public:

    EnvironmentCARTROBARM3D(OccupancyGrid *grid, SBPLKinematicModel *kmodel, CollisionChecker *cc);

    ~EnvironmentCARTROBARM3D();
    
    bool setStartConfiguration(std::vector<double> angles);

    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);

    void StateID2Angles(int stateID, std::vector<double> &angles);

    bool setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances);

    void convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);
    
    void convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short);

    void convertStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);

    //TODO: Will not work with multiple waypoint motion prims...It's just for fast debugging.
    void convertShortStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);

    void getContMotionPrims(char type, std::vector<std::vector<btVector3> > &mprims);

    void printEnvironmentStats();

    void computeGradient(const MotionPrimitive &mp, unsigned char &d, bool collision);

    std::vector<double> mp_gradient_;
    std::vector<double> mp_dist_;

    int getXYZRPYHeuristic(int FromStateID, int ToStateID);

  protected:
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV=NULL );

  private:

    int free_angle_index_;
    int ndof_;

    EnvROBARM3DHashEntry_t* createHashEntry(const std::vector<int> &coord);
    EnvROBARM3DHashEntry_t* getHashEntry(const std::vector<int> &coord, bool bIsGoal);
    EnvROBARM3DHashEntry_t* getHashEntry(int *xyz, int *rpy, int fangle, bool bIsGoal);

    /** initialization */
    bool initEnvConfig();

    /* discretization */
    void coordToPose(const std::vector<int> &coord, int *xyz, int *rpy, int *fangle);
    void coordToWorldPose(const std::vector<int> &coord, double *xyz, double *rpy, double *fangle);
    void coordToWorldPose(const std::vector<int> &coord, std::vector<double> &wcoord);
    void anglesToCoord(const std::vector<double> &angles, std::vector<int> &coord);
    void stateIDToPose(int stateID, int *xyz, int *rpy, int *fangle);
    void stateIDToWorldPose(int stateID, double *wxyz, double *wrpy, double *wfangle);
    void worldPoseToCoord(double *wxyz, double *wrpy, double wfangle, std::vector<int> &coord);
    void worldPoseToState(double *wxyz, double *wrpy, double wfangle, bool is_goal, EnvROBARM3DHashEntry_t *state);
    void coordToAngles(const std::vector<int> &coord, std::vector<double> &angles);
    void discToWorldXYZ(int *xyz, double *wxyz);
    void discToWorldRPY(int *rpy, double *wrpy);
    void discToWorldFAngle(int fangle, double *wfangle);
    void worldToDiscXYZ(double *wxyz, int *xyz);
    void worldToDiscRPY(double *wrpy, int *rpy);
    void worldToDiscFAngle(double wfangle, int *fangle);
    bool convertCoordToAngles(const std::vector<int> *coord, std::vector<double> *angles);
    bool convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> &angles);
    bool convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> seed, std::vector<double> &angles);

    bool isGoalPosition(double *xyz, double *rpy, double fangle);
    bool isGoalPosition(const std::vector<int> &coord);
    void getAdaptiveMotionPrim(int type, EnvROBARM3DHashEntry_t* parent, MotionPrimitive &mp);
    int getJointAnglesForMotionPrimWaypoint(const std::vector<double> &mp_point, const std::vector<double> &wcoord, const std::vector<double> &pangles, std::vector<double> &final_wcoord, std::vector<std::vector<double> > &angles);
    bool getMotionPrimitive(EnvROBARM3DHashEntry_t* parent, MotionPrimitive &mp);
    int isMotionValid(const std::vector<double> &start, const std::vector<double> &end, int &path_length, int &nchecks, unsigned char &dist);
    void getVector(int x1, int y1, int z1, int x2, int y2, int z2, int &xout, int &yout, int &zout, int multiplier, bool snap=true);
    bool getDistanceGradient(int &x, int &y, int &z);
    void computeCostPerCell();
    int computeMotionCost(const std::vector<double> &a, const std::vector<double> &b);
};

inline void EnvironmentCARTROBARM3D::discToWorldXYZ(int *xyz, double *wxyz)
{
  grid_->gridToWorld(int(xyz[0]),int(xyz[1]),int(xyz[2]),wxyz[0],wxyz[1],wxyz[2]);
  ROS_DEBUG("[discToWorldXYZ] xyz: %d %d %d --> %2.3f %2.3f %2.3f",xyz[0],xyz[1],xyz[2],wxyz[0],wxyz[1],wxyz[2]);
}

inline void EnvironmentCARTROBARM3D::worldToDiscXYZ(double *wxyz, int *xyz)
{
  int x,y,z;
  grid_->worldToGrid(wxyz[0],wxyz[1],wxyz[2],x,y,z);
  xyz[0] = x;
  xyz[1] = y;
  xyz[2] = z;
  ROS_DEBUG("[worldToDiscXYZ] xyz: %2.3f %2.3f %2.3f --> %d %d %d",wxyz[0],wxyz[1],wxyz[2],xyz[0],xyz[1],xyz[2]);
}

inline void EnvironmentCARTROBARM3D::discToWorldRPY(int *rpy, double *wrpy)
{
  wrpy[0] = angles::normalize_angle(double(rpy[0])*EnvROBARMCfg.coord_delta[3]);
  wrpy[1] = angles::normalize_angle(double(rpy[1])*EnvROBARMCfg.coord_delta[4]);
  wrpy[2] = angles::normalize_angle(double(rpy[2])*EnvROBARMCfg.coord_delta[5]);

  ROS_DEBUG("[discToWorldRPY] rpy: %d %d %d --> %2.3f %2.3f %2.3f",rpy[0],rpy[1],rpy[2],wrpy[0],wrpy[1],wrpy[2]);
}

inline void EnvironmentCARTROBARM3D::worldToDiscRPY(double *wrpy, int *rpy)
{
  rpy[0] = (int)((angles::normalize_angle_positive(wrpy[0]) + EnvROBARMCfg.coord_delta[3]*0.5)/EnvROBARMCfg.coord_delta[3]) % EnvROBARMCfg.coord_vals[3];
  rpy[1] = (int)((angles::normalize_angle_positive(wrpy[1]) + EnvROBARMCfg.coord_delta[4]*0.5)/EnvROBARMCfg.coord_delta[4]) % EnvROBARMCfg.coord_vals[4];
  rpy[2] = (int)((angles::normalize_angle_positive(wrpy[2]) + EnvROBARMCfg.coord_delta[5]*0.5)/EnvROBARMCfg.coord_delta[5]) % EnvROBARMCfg.coord_vals[5];


  ROS_DEBUG("[worldToDiscRPY] rpy: %2.3f %2.3f %2.3f --> %d %d %d",wrpy[0],wrpy[1],wrpy[2],rpy[0],rpy[1],rpy[2]);

  if(rpy[0] >= EnvROBARMCfg.coord_vals[3] || rpy[1] >= EnvROBARMCfg.coord_vals[4] || rpy[2] >= EnvROBARMCfg.coord_vals[5])
  {
    ROS_ERROR("[worldToDiscRPY] wrpy: %0.3f %0.3f %0.3f discretized to %d %d %d", wrpy[0], wrpy[1], wrpy[2], rpy[0], rpy[1], rpy[2]);
  }
}

inline void EnvironmentCARTROBARM3D::discToWorldFAngle(int fangle, double *wfangle)
{
  *wfangle = angles::normalize_angle(double(fangle)*EnvROBARMCfg.coord_delta[6]);

  ROS_DEBUG("[discToWorldFAngle] fangle: %d --> %2.3f",fangle,*wfangle);
}

inline void EnvironmentCARTROBARM3D::worldToDiscFAngle(double wfangle, int *fangle)
{
  *fangle = (int)((angles::normalize_angle_positive(wfangle) + EnvROBARMCfg.coord_delta[6]*0.5)/EnvROBARMCfg.coord_delta[6]) % EnvROBARMCfg.coord_vals[6];

  ROS_DEBUG("[worldToDiscFAngle] fangle: %2.3f --> %d",wfangle,*fangle);

  if(*fangle >= EnvROBARMCfg.coord_vals[6])
    ROS_ERROR("[worldToDiscFAngle] fangle: %0.3f discretized to: %d", wfangle, *fangle);
}

inline void EnvironmentCARTROBARM3D::coordToPose(const std::vector<int> &coord, int *xyz, int *rpy, int *fangle)
{
  xyz[0] = coord[0];
  xyz[1] = coord[1];
  xyz[2] = coord[2];
  rpy[0] = coord[3];
  rpy[1] = coord[4];
  rpy[2] = coord[5];
  *fangle = coord[6];

  ROS_DEBUG("[coordToPose] xyz: %u %u %u  rpy: %d %d %d fangle: %u", xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2],*fangle);
}

inline void EnvironmentCARTROBARM3D::coordToWorldPose(const std::vector<int> &coord, double *wxyz, double *wrpy, double *wfangle)
{
  int xyz[3]={0}, rpy[3]={0}, fangle=0;
  
  coordToPose(coord,xyz,rpy,&fangle);

  discToWorldXYZ(xyz,wxyz);
  discToWorldRPY(rpy,wrpy);
  discToWorldFAngle(fangle,wfangle);
}

inline void EnvironmentCARTROBARM3D::coordToWorldPose(const std::vector<int> &coord, std::vector<double> &wcoord)
{
  double xyz[3]={0}, rpy[3]={0}, fangle=0;
  
  coordToWorldPose(coord, xyz, rpy, &fangle);

  wcoord[0] = xyz[0];
  wcoord[1] = xyz[1];
  wcoord[2] = xyz[2];
  wcoord[3] = rpy[0];
  wcoord[4] = rpy[1];
  wcoord[5] = rpy[2]; 
  wcoord[6] = fangle;
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentCARTROBARM3D::coordToAngles(const std::vector<int> &coord, std::vector<double> &angles)
{
  EnvROBARM3DHashEntry_t* h;
  if((h = getHashEntry(coord,false)) == NULL)
  {
    if(!isGoalPosition(coord) || (h = getHashEntry(coord,true)) == NULL)
    {
      ROS_WARN("[coordToAngles] Failed to fetch hash entry (coord: %d %d %d %d %d %d %d).", coord[0], coord[1], coord[2], coord[3], coord[4], coord[5], coord[6]);
      return;
    }
  }

  angles = h->angles;
}

} //namespace

#endif

