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

#ifndef _PLANNING_PARAMS_H_
#define _PLANNING_PARAMS_H_

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <angles/angles.h>
#include <sstream>
#include <boost/algorithm/string.hpp>

using namespace std;

namespace sbpl_arm_planner {


class PlanningParams
{
  public:

    PlanningParams();
    ~PlanningParams(){};

    bool init(std::string ns="~");

    void printParams(std::string stream);

    /* Search */
    bool search_mode_;
    bool shortcut_path_;
    bool interpolate_path_;
    double allowed_time_;
    double waypoint_time_;

    /* Planning */
    bool ready_to_plan_;
    int num_joints_;
    std::string planning_frame_;
    std::string group_name_;
    std::string planner_name_;
    std::string statespace_type_;
    std::vector<std::string> planning_joints_;

    /* Options */
    bool use_bfs_heuristic_;
    double epsilon_;
    double planning_link_sphere_radius_;

    /* Discretization */
    std::vector<int> coord_vals_;
    std::vector<double> coord_delta_;

    /* Costs */
    int cost_multiplier_;
    int cost_per_cell_;
    int cost_per_meter_;
    int cost_per_second_;
    double time_per_cell_;
    double max_mprim_offset_;
    
    /* Debugging & Logging */
    bool print_path_;
    bool verbose_;
    bool verbose_heuristics_;
    bool verbose_collisions_;
    std::string expands_log_;
    std::string expands2_log_;
    std::string ik_log_;
    std::string rmodel_log_;
    std::string cspace_log_;
    std::string solution_log_;
    std::string expands_log_level_;
    std::string expands2_log_level_;
    std::string ik_log_level_;
    std::string rmodel_log_level_;
    std::string cspace_log_level_;
    std::string solution_log_level_;
};

}
#endif

