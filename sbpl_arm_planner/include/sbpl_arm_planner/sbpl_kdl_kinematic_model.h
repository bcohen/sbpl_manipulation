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

#ifndef _SBPL_KDL_KINEMATIC_MODEL_
#define _SBPL_KDL_KINEMATIC_MODEL_

#include <string>
#include <vector>
#include <ros/console.h>
#include <angles/angles.h>
#include <urdf/model.h>
#include <sbpl_arm_planner/sbpl_kinematic_model.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

//#include <Eigen/Core>

using namespace std;

namespace sbpl_arm_planner {

class SBPLKDLKinematicModel : public SBPLKinematicModel {

  public:

    SBPLKDLKinematicModel();
    
    ~SBPLKDLKinematicModel();
   
    /* Initialization */
    virtual bool init(std::string robot_description);

    /* Joint Limits */
    virtual bool checkJointLimits(const std::vector<double> &angles, bool verbose);
   
    /* Forward Kinematics */
    virtual bool computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f);

    virtual bool computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose);

    virtual bool computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose);

    /* Inverse Kinematics */
    virtual bool computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution);

    virtual bool computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution);

    /* Debug Output */
    virtual void printKinematicModelInformation(std::string stream);

  private:

    urdf::Model robot_model_;

    std::string chain_root_name_;
    std::string chain_tip_name_;

    KDL::Tree ktree_;
    KDL::Chain kchain_;
    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::ChainFkSolverPos_recursive *fk_solver_;
    KDL::ChainIkSolverVel_pinv *ik_vel_solver_;
    KDL::ChainIkSolverVel_NR *ik_solver_;

};

}
#endif
