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

#include <sbpl_manipulation_components_pr2/ubr1_kdl_robot_model.h>
#include <ros/ros.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <kdl/tree.hpp>

using namespace std;

namespace sbpl_arm_planner {

UBR1KDLRobotModel::UBR1KDLRobotModel() : rpy_solver_(NULL)
{
  chain_root_name_ = "torso_lift_link";
  chain_tip_name_ = "gripper_link";
  forearm_roll_link_name_ = "forearm_roll_link";
  wrist_pitch_joint_name_ = "wrist_flex_joint";
  end_effector_link_name_ = "gripper_link";

  // initialize rpy solver
  double wrist_max_limit=-0.015, wrist_min_limit=-2.0;
  bool wrist_continuous;
  //if(!getJointLimits(wrist_pitch_joint_name_, wrist_min_limit, wrist_max_limit, wrist_continuous))
  //  ROS_ERROR("Failed to get wrist pitch joint limits...");
  //ROS_ERROR("Wrist limits:  {%0.3f, %0.3f}", wrist_min_limit, wrist_max_limit);
  rpy_solver_ = new sbpl_arm_planner::RPYSolver(wrist_min_limit, wrist_max_limit);
}

UBR1KDLRobotModel::~UBR1KDLRobotModel()
{
  if(rpy_solver_)
    delete rpy_solver_;
}

/*
bool UBR1KDLRobotModel::init(std::string robot_description, std::vector<std::string> &planning_joints)
{

  initialized_ = true;
  return true;
}
*/

bool UBR1KDLRobotModel::computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, int option)
{
  //pose: {x,y,z,r,p,y} or {x,y,z,qx,qy,qz,qw}
  KDL::Frame frame_des;
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);

  // RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  // quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  // transform into kinematics frame
  frame_des = T_planning_to_kinematics_ * frame_des;

  // seed configuration
  for(size_t i = 0; i < start.size(); i++)
    jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJntSearch

  solution.resize(start.size());

  // choose solver
  if(option == sbpl_arm_planner::ik_option::RESTRICT_XYZ_JOINTS)
  {
    std::vector<double> rpy(3,0), fpose(6,0), epose(6,0);
    frame_des.M.GetRPY(rpy[0], rpy[1], rpy[2]);
    std::vector<double> const rpy2(rpy);

    // get pose of forearm link
    if(!computeFK(start, forearm_roll_link_name_, fpose))
    {
      ROS_ERROR("[rm] computeFK failed on forearm pose.");
      return false;
    }

    // get pose of end-effector link
    if(!computeFK(start, end_effector_link_name_, epose))
    {
      ROS_ERROR("[rm] computeFK failed on end_eff pose.");
      return false;
    }

    return rpy_solver_->computeRPYOnly(rpy2, start, fpose, epose, 1, solution);
  }
  else
  {
    if(computeIKSearch(pose, start, solution, 0.01) < 0)
      return false;
    
    for(size_t i = 0; i < solution.size(); ++i)
      solution[i] = jnt_pos_out_(i);
  }

  return true;
}

}
