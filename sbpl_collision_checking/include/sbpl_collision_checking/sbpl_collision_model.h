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
#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <sbpl_collision_checking/group.h>
#include <arm_navigation_msgs/MultiDOFJointState.h>

namespace sbpl_arm_planner
{

class SBPLCollisionModel
{
  public:
    
    SBPLCollisionModel();

    ~SBPLCollisionModel();

    bool init(std::string ns="");

    bool initAllGroups();

    void getGroupNames(std::vector<std::string> &names);

    bool getJointLimits(std::string group_name, std::string joint_name, double &min_limit, double &max_limit, bool &continuous);

    bool setDefaultGroup(std::string group_name);

    void getDefaultGroupSpheres(std::vector<Sphere*> &spheres, bool low_res=false);

    void getVoxelGroups(std::vector<Group*> &vg);

    void getSphereGroups(std::vector<Group*> &vg);

    std::vector<Sphere*> getGroupSpheres(std::string group_name, bool low_res);

    bool computeDefaultGroupFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame> > &frames);

    bool computeGroupFK(const std::vector<double> &angles, Group* group, std::vector<std::vector<KDL::Frame> > &frames);

    void setOrderOfJointPositions(const std::vector<std::string> &joint_names, std::string group_name);

    void setJointPosition(const std::string &name, double position);

    bool getFrameInfo(std::string &name, std::string group_name, int &chain, int &segment);

    bool doesLinkExist(std::string name, std::string group_name);

    std::string getReferenceFrame(std::string group_name);

    Group* getGroup(std::string name);

    Group* getDefaultGroup() {return dgroup_;};

    void printGroups();
    
    void printDebugInfo(std::string group_name);

    bool setModelToWorldTransform(const arm_navigation_msgs::MultiDOFJointState &state, std::string world_frame);

    void setSphereGroupsForCheckCollision(const std::vector<std::string> &group_names);

    bool doLinkToLinkCheck(int link1, int link2);
  
  private:

    ros::NodeHandle nh_, ph_;

    std::map<std::string, Group*> group_config_map_;

    std::map<std::string, Link*> link_map_;
    
    boost::shared_ptr<urdf::Model> urdf_;
    
    Group* dgroup_;

    std::vector<Group*> sphere_groups_;

    std::vector<std::vector<bool> > collision_matrix_;

    bool getRobotModel();

    bool readGroups(std::string ns="");
    
    bool readCollisionMatrix(std::string ns="");
   
    bool computeFK(const std::vector<double> &angles, Group* group, int chain, int segment, KDL::Frame &frame);
};

}
#endif

