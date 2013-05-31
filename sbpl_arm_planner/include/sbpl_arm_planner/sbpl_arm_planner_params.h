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

#ifndef _SBPL_ARM_PLANNER_PARAMS_H_
#define _SBPL_ARM_PLANNER_PARAMS_H_

#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <ros/ros.h>
#include <angles/angles.h>
#include <sstream>
#include <boost/algorithm/string.hpp>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

using namespace std;

namespace sbpl_arm_planner {

enum MotionPrimitiveType {
  LONG_DISTANCE,
  SHORT_DISTANCE,
  ADAPTIVE,
  SNAP_TO_RPY,
  SNAP_TO_XYZRPY,
  SNAP_TO_RPY_THEN_TO_XYZ,
  SNAP_TO_XYZ_THEN_TO_RPY,
  SNAP_TO_RPY_AT_START,
  RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ,
  RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ,
  NUMBER_OF_MPRIM_TYPES
};

//static const char* MotionPrimitiveTypeNames[] = {"long_distance","short_distance", "adaptive", "snap_to_rpy", "snap_to_xyzrpy",
//                                                            "snap_to_rpy_then_to_xyz", "snap_to_xyz_then_to_rpy"};

typedef std::vector<std::vector<double> > MPrim;
typedef std::vector<int> Coords;

typedef struct
{
  char type;
  int nsteps;
  int id;
  int group;
  MPrim m;
  Coords coord;
} MotionPrimitive;

class SBPLArmPlannerParams
{
  public:

    /** \brief default constructor (just assigns default values) */
    SBPLArmPlannerParams();

    /** \brief destructor */
    ~SBPLArmPlannerParams(){};

    /** \brief grab parameters from the ROS param server */
    void initFromParamServer();

    /** \brief grab parameters from a text file */
//    bool initFromParamFile(std::string param_file);
    
    /** \brief grab motion primitives from a text file */
    bool initMotionPrimsFromFile(FILE* fCfg);

    /** \brief grab long primitives from a text file  (cartesian arm planner)*/
    bool initLongMotionPrimsFromFile(FILE* fCfg);

    /** \brief grab parameters from a text file */
//    bool initFromParamFile(FILE* fCfg);

    /** \brief print out the parameters */
    void printParams(std::string stream);

    /** \brief add a motion primitive vector to the list */
    void addMotionPrim(std::vector<double> mprim, bool add_converse, bool short_dist_mprim);

    /** \brief set the cost per cell */
    void setCellCost(int cost_per_cell);

    /** \brief print the motion primitives */
    void printMotionPrims(std::string stream);

//    double getSmallestShoulderPanMotion();

    /** \brief NOTE: For computing cost for IK and OS solutions */
    double getLargestMotionPrimOffset();

    /** \brief Output the long motion prims to the terminal */
    void printLongMotionPrims(std::string stream);

    bool initMotionPrims(FILE* fCfg);

    void printMotionPrim(MotionPrimitive mp, std::string text);

    void changeLoggerLevel(std::string name, std::string level);

    /** \brief epsilon to be used by planner (epsilon is the bounds on the
     * suboptimality of the solution of a weighted A* type search)*/
    double epsilon_;

    /** \brief enable multi-resolution motion primitives */
    bool use_multires_mprims_;

    /** \brief use a 3D dijkstra search as the heuristic */
    bool use_bfs_heuristic_;

    /** \brief use the orientation solver to try to satisfy orientation constraint */
    bool use_orientation_solver_;

    /** \brief use IK to try to satisfy the orientation constraint */
    bool use_ik_;

    /** \brief plan to a 6D pose goal (if false, plans to 3D goal {x,y,z}) */
    bool use_6d_pose_goal_;

    /** \brief add the h_rpy & h_xyz together */
    bool sum_heuristics_;

    /** \brief uniform cost when moving from one cell to another (if false,
     * a cost will be applied based on distance from nearest obstacle) */
    bool use_uniform_cost_;

    /** \brief spew out all debugging text */
    bool verbose_;

    /** \brief spew out all heuristic related debugging text */
    bool verbose_heuristics_;

    /** \brief spew out all collision checking related debugging text */
    bool verbose_collisions_;

    /** \brief discretization of joint angles (default: 360)*/
    int angle_delta_;

    /** \brief the 2D array of motion primitives */
    std::vector<std::vector<double> > mprims_;

    /** \brief total number of motion primitives */
    int num_mprims_;

    /** \brief number of long distance motion primitives */
    int num_long_dist_mprims_;

    /** \brief number of short distance motion primitives */
    int num_short_dist_mprims_;

    /** \brief distance from goal pose in cells to switch to using short distance
     * motion primitives + long distance motion primitives */
    int short_dist_mprims_thresh_c_;

    /** \brief distance from goal pose in meters to switch to using short distance
     * motion primitives + long distance motion primitives */
    double short_dist_mprims_thresh_m_;

    std::string planner_name_;

    std::string environment_type_;  // "jointspace" or "cartesian"

    /** \brief cost multiplier (so we don't have to deal with doubles) */
    int cost_multiplier_;

    int range1_cost_;
    int range2_cost_;
    int range3_cost_;

    /** \brief cost of moving one cell in the grid */
    int cost_per_cell_;

    /** \brief cost of moving one meter (only used when euclidean distance is
     * used as the heuristic instead of dijkstra) */
    int cost_per_meter_;

    /** \brief set which function to use to check if at goal position */
    int is_goal_function_;

    /** \brief call the orientation solver twice (try rotating the wrist clockwise and then counter-clockwise) */
//    bool two_calls_to_op_;

    /** \brief use the elbow heuristic */
    bool use_research_heuristic_;
    
    double max_mprim_offset_;

    /** \brief size of collision space */
    double sizeX_;
    double sizeY_;
    double sizeZ_;

    /** \brief resolution of collision space */
    double resolution_;

    /** \brief origin of collision space */
    double originX_;
    double originY_;
    double originZ_;

    int solve_for_ik_thresh_;
    double solve_for_ik_thresh_m_;

    std::string reference_frame_;

    /* For Cartesian Arm Planner */
    std::vector<MotionPrimitive> mp_;
    double xyz_resolution_;
    double rpy_resolution_;
    double fa_resolution_;

    int cost_per_second_;
    double time_per_cell_;
    std::vector<double> joint_vel_;

    std::string group_name_;
    std::vector<std::string> planning_joints_;


    std::string expands_log_;
    std::string expands2_log_;
    std::string ik_log_;
    std::string arm_log_;
    std::string cspace_log_;
    std::string solution_log_;
    std::string expands_log_level_;
    std::string expands2_log_level_;
    std::string ik_log_level_;
    std::string arm_log_level_;
    std::string cspace_log_level_;
    std::string solution_log_level_;

    std::vector<std::string> motion_primitive_type_names_;
};

}
#endif

