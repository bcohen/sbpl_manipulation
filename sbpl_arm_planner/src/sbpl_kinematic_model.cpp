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
 /** \author Benjamin Cohen */

#include <sbpl_arm_planner/sbpl_kinematic_model.h>

using namespace std;

namespace sbpl_arm_planner {

SBPLKinematicModel::SBPLKinematicModel()
{
  logger_ = "kinematic_model";
  initialized_ = false;
}

bool SBPLKinematicModel::init(std::string robot_description)
{
  initialized_ = true;
  return true;
}

void SBPLKinematicModel::setLoggerName(std::string name)
{
  logger_ = name;
}

void SBPLKinematicModel::setPlanningJoints(const std::vector<std::string> &joints)
{
  planning_joints_ = joints;
}

void SBPLKinematicModel::setPlanningLink(std::string name)
{
  planning_link_ = name;
}

void SBPLKinematicModel::setPlanningFrame(std::string name)
{
  planning_frame_ = name;
}

void SBPLKinematicModel::getKinematicsFrame(std::string &name)
{
  name = kinematics_frame_;
}

bool SBPLKinematicModel::computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f)
{
  ROS_ERROR("Function not filled in.");  
  return false;
}

bool SBPLKinematicModel::computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose)
{
  ROS_ERROR("Function not filled in.");  
  return false;
}

bool SBPLKinematicModel::computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution)
{
  ROS_ERROR("Function not filled in."); 
  return false;
}

bool SBPLKinematicModel::computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution)
{
  ROS_ERROR("Function not filled in.");  
  return false;
}

void SBPLKinematicModel::printKinematicModelInformation(std::string stream)
{
  ROS_ERROR("Function not filled in.");  
}

bool SBPLKinematicModel::checkJointLimits(const std::vector<double> &angles, bool verbose)
{
  ROS_ERROR("Function not filled in.");  
  return false;
}

void SBPLKinematicModel::setKinematicsToPlanningTransform(const KDL::Frame &f, std::string name)
{
  T_kinematics_to_planning_ = f;
  T_planning_to_kinematics_ = f.Inverse();
  planning_frame_ = name;
}


}
