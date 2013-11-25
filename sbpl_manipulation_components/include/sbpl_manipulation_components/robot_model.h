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

#ifndef _ROBOT_MODEL_
#define _ROBOT_MODEL_

#include <string>
#include <vector>
#include <ros/console.h>
#include <angles/angles.h>
#include <kdl/frames.hpp>

using namespace std;

namespace sbpl_arm_planner {

namespace ik_option {
  enum
  {
    UNRESTRICTED,
    RESTRICT_XYZ_JOINTS
  };
}

class RobotModel {

  public:

    RobotModel();
    
    ~RobotModel(){};
   
    /* Initialization */
    virtual bool init(std::string robot_description, std::vector<std::string> &planning_joints);

    void setPlanningJoints(const std::vector<std::string> &joints);

    void setPlanningLink(std::string name);

    std::string getPlanningLink();

    void setPlanningFrame(std::string name);

    std::string getPlanningFrame();

    void getKinematicsFrame(std::string &name);

    /* Joint Limits */
    virtual bool checkJointLimits(const std::vector<double> &angles);
   
    double getMaxJointLimit(std::string name);

    double getMinJointLimit(std::string name);

    /* Forward Kinematics */
    virtual bool computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f);

    virtual bool computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose);

    virtual bool computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose);

    /* Inverse Kinematics */
    virtual bool computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, int option=0);

    virtual bool computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution);

    /* Debug Output */
    virtual void printRobotModelInformation();
    
    void setLoggerName(std::string name);

    /* Transform between Kinematics frame <-> Planning frame */
    void setKinematicsToPlanningTransform(const KDL::Frame &f, std::string name);


  protected:

    bool initialized_;

    /** \brief a string containing the URDF that describes the robot arm */
    std::string robot_description_;

    /** \brief frame that the planning is done in (i.e. map) */
    std::string planning_frame_;

    /** \brief frame that the kinematics is computed in (i.e. robot base) */
    std::string kinematics_frame_;

    /** \brief the link that is being planned for (i.e. wrist) */
    std::string planning_link_;

    std::vector<std::string> planning_joints_;

    KDL::Frame T_kinematics_to_planning_;
    
    KDL::Frame T_planning_to_kinematics_;
   
    /** \brief ROS logger stream name */ 
    std::string logger_;
};

}
#endif
