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

#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <ros/ros.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <kdl/tree.hpp>

using namespace std;

namespace sbpl_arm_planner {

PR2KDLRobotModel::PR2KDLRobotModel() : pr2_ik_solver_(NULL), rpy_solver_(NULL)
{
  chain_root_name_ = "torso_lift_link";
  chain_tip_name_ = "r_gripper_palm_link";
  forearm_roll_link_name_ = "r_forearm_roll_link";
  wrist_pitch_joint_name_ = "r_wrist_flex_joint";
  end_effector_link_name_ = "r_gripper_palm_link";
}

PR2KDLRobotModel::~PR2KDLRobotModel()
{
  if(pr2_ik_solver_)
    delete pr2_ik_solver_;

  if(rpy_solver_)
    delete rpy_solver_;
}

bool PR2KDLRobotModel::init(std::string robot_description, std::vector<std::string> &planning_joints)
{
  urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
  if(!urdf_->initString(robot_description))
  {
    ROS_ERROR("Failed to parse the URDF.");
    return false;
  }
 
  if(!kdl_parser::treeFromUrdfModel(*urdf_, ktree_))
  {
    ROS_ERROR("Failed to parse the kdl tree from robot description.");
    return false;
  }

  std::vector<std::string> segments(planning_joints.size());
  for(size_t j = 0; j < planning_joints.size(); ++j)
  {
    if(!leatherman::getSegmentOfJoint(ktree_, planning_joints[j], segments[j]))
    {
      ROS_ERROR("Failed to find kdl segment for '%s'.", planning_joints_[j].c_str());
      return false;
    }
  }

  if(!ktree_.getChain(chain_root_name_, chain_tip_name_, kchain_))
  {
    ROS_ERROR("Failed to fetch the KDL chain for the robot. (root: %s, tip: %s)", chain_root_name_.c_str(), chain_tip_name_.c_str());
    return false;
  }

  // check if our chain includes all planning joints
  for(size_t i = 0; i < planning_joints.size(); ++i)
  {
    if(planning_joints[i].empty())
    {
      ROS_ERROR("Planning joint name is empty (index: %d).", int(i));
      return false;
    }
    int index;
    if(!leatherman::getJointIndex(kchain_, planning_joints[i], index))
    {
      ROS_ERROR("Failed to find '%s' in the kinematic chain. Maybe your chain root or tip joints are wrong? (%s, %s)", planning_joints[i].c_str(), chain_root_name_.c_str(), chain_tip_name_.c_str());
      return false;
    }
  }

  // joint limits
  planning_joints_ = planning_joints;
  if(!getJointLimits(planning_joints_, min_limits_, max_limits_, continuous_))
  {
    ROS_ERROR("Failed to get the joint limits.");
    return false;
  }

  // FK solver
  fk_solver_ = new KDL::ChainFkSolverPos_recursive(kchain_);
  jnt_pos_in_.resize(kchain_.getNrOfJoints());
  jnt_pos_out_.resize(kchain_.getNrOfJoints());

  // IK solver
  KDL::JntArray q_min(planning_joints_.size());
  KDL::JntArray q_max(planning_joints_.size());
  for(size_t i = 0; i < planning_joints_.size(); ++i)
  {
    q_min(i) = min_limits_[i];
    q_max(i) = max_limits_[i];
  }
  ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(kchain_);
  ik_solver_ = new KDL::ChainIkSolverPos_NR_JL(kchain_, q_min, q_max, *fk_solver_, *ik_vel_solver_, 200, 0.001);

  // PR2 Specific IK Solver
  pr2_ik_solver_ = new pr2_arm_kinematics::PR2ArmIKSolver(*urdf_, chain_root_name_, chain_tip_name_, 0.02, 2);
  if(!pr2_ik_solver_->active_)
  {
    ROS_ERROR("The pr2 IK solver is NOT active. Exiting.");
    return false;
  }

  // joint name -> index mapping
  for(size_t i = 0; i < planning_joints_.size(); ++i)
    joint_map_[planning_joints_[i]] = i;

  // TODO: figure out why the link_map_ can be initialized incorrectly in
  // some cases causing it to seg fault in the next for loop.
  link_map_ = std::map<std::string, int>();

  // link name -> kdl index mapping
  for(size_t i = 0; i < kchain_.getNrOfSegments(); ++i)
    link_map_[kchain_.getSegment(i).getName()] = i;

  // initialize rpy solver
  double wrist_min_limit, wrist_max_limit;
  bool wrist_continuous;
  if(!getJointLimits(wrist_pitch_joint_name_, wrist_min_limit, wrist_max_limit, wrist_continuous))
    return false;
  rpy_solver_ = new sbpl_arm_planner::RPYSolver(wrist_min_limit, wrist_max_limit);

  initialized_ = true;
  return true;
}

bool PR2KDLRobotModel::computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, int option)
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
    if(pr2_ik_solver_->CartToJntSearch(jnt_pos_in_, frame_des, jnt_pos_out_, 0.2) < 0)
      return false;
    
    for(size_t i = 0; i < solution.size(); ++i)
      solution[i] = jnt_pos_out_(i);
  }

  return true;
}

bool PR2KDLRobotModel::computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution)
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

  if(pr2_ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = jnt_pos_out_(i);

  return true;
}

void PR2KDLRobotModel::printRobotModelInformation()
{
  leatherman::printKDLChain(kchain_, "robot_model");

  ROS_INFO("Joint<->Index Map:");
  for(std::map<std::string, int>::const_iterator iter = joint_map_.begin(); iter != joint_map_.end(); ++iter)
     ROS_INFO("%22s: %d", iter->first.c_str(), iter->second);

  ROS_INFO("Link<->KDL_Index Map:");
  for(std::map<std::string, int>::const_iterator iter = link_map_.begin(); iter != link_map_.end(); ++iter)
     ROS_INFO("%22s: %d", iter->first.c_str(), iter->second);
}

}
