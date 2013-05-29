/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef _SBPL_COLLISION_MODEL_
#define _SBPL_COLLISION_MODEL_

#include <ros/ros.h>
#include <iostream>
#include <map>
#include <vector>
#include <sstream>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace sbpl_arm_planner
{

  struct Sphere
  { 
    std::string name;
    KDL::Vector v;
    double radius;
    int priority;
    int kdl_chain;
    int kdl_segment;
  };

  struct Link
  {
    int i_chain_;
    std::string name_;
    std::string root_name_;
    std::vector<Sphere> spheres_;
  };

  struct Group
  {
    Group(std::string name) : name_(name)
    {
      init = false;
    }

    ~Group()
    {
      for(size_t i = 0; i < solvers_.size(); i++)
      {
        if(solvers_[i] != NULL)
        {
          delete solvers_[i];
          solvers_[i] = NULL;
        }
      }
    }

    bool init; // TODO: Add underscore
    std::string name_;
    std::string root_name_;
    std::string tip_name_;
    std::string redundancy_name_; 
    KDL::Chain chain_;
    std::vector<KDL::Chain> chains_;
    std::vector<KDL::ChainFkSolverPos_recursive*> solvers_;
    std::vector<KDL::JntArray> joint_positions_;
    std::vector<std::vector<int> > frames_;
    std::vector<std::vector<std::string> > jntarray_names_;
    std::vector<std::vector<int> > angles_to_jntarray_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> order_of_input_angles_;
    std::vector<Link> links_;
    std::vector<Sphere*> spheres_;
  };


class SBPLCollisionModel
{
  public:
    
    SBPLCollisionModel();
    ~SBPLCollisionModel();

    bool init();

    bool initGroup(Group* group);

    bool initGroup(std::string group_name);

    bool initAllGroups();

    void getGroupNames(std::vector<std::string> &names);

    bool getJointLimits(std::string root_name, std::string tip_name, std::vector<std::string> &joint_names, std::vector<double> min_limits, std::vector<double> &max_limits, std::vector<bool> &continuous);

    bool getJointLimits(std::string group_name, std::string joint_name, double &min_limit, double &max_limit, bool &continuous);

    void setDefaultGroup(std::string group_name);

    void getDefaultGroupSpheres(std::vector<Sphere*> &spheres);

    bool computeDefaultGroupFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame> > &frames);

    bool computeGroupFK(const std::vector<double> &angles, Group* group, std::vector<std::vector<KDL::Frame> > &frames);

    void setOrderOfJointPositions(const std::vector<std::string> &joint_names, std::string group_name);

    void setJointPosition(const std::string &name, double position);

    bool getFrameInfo(std::string &name, std::string group_name, int &chain, int &segment);

    std::string getReferenceFrame(std::string group_name);

    void printGroups();
    void printSpheres(std::string group_name);
    void printChain(KDL::Chain &chain);
    void printDebugInfo(std::string group_name);
    void printFrames(const std::vector<std::vector<KDL::Frame> > &f);

  private:

    ros::NodeHandle nh_, ph_;

    std::map< std::string, Group*> group_config_map_;
    
    boost::shared_ptr<urdf::Model> urdf_;
    
    KDL::Tree kdl_tree_;

    Group* dgroup_;

    bool getRobotModel();

    void readGroups();
    
    void sortSpheres(std::string group_name);

    bool initKDLChainForGroup(std::string &chain_root_name, Group* group);

    bool initKDLChainForGroup(Group* group);

    void initFKSolvers(Group* group);

    int getSegmentNumber(std::string &name, KDL::Chain &chain);

    bool computeFK(const std::vector<double> &angles, Group* group, int chain, int segment, KDL::Frame &frame);
};

}
#endif

