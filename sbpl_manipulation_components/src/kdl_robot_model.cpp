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

#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <ros/ros.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <sbpl_geometry_utils/utils.h>
#include <kdl/tree.hpp>

using namespace std;

namespace sbpl_arm_planner {

KDLRobotModel::KDLRobotModel() : ik_solver_(NULL), ik_vel_solver_(NULL), fk_solver_(NULL)
{
  ros::NodeHandle ph("~");
  ph.param<std::string>("robot_model/chain_root_link", chain_root_name_, " ");
  ph.param<std::string>("robot_model/chain_tip_link", chain_tip_name_, " ");
  ph.param("robot_model/free_angle", free_angle_, 2);
}

KDLRobotModel::KDLRobotModel(std::string chain_root_link, std::string chain_tip_link) : ik_solver_(NULL), ik_vel_solver_(NULL), fk_solver_(NULL)
{
  free_angle_ = 2;
  chain_root_name_ = chain_root_link;
  chain_tip_name_ = chain_tip_link;
}

KDLRobotModel::~KDLRobotModel()
{
  if(ik_solver_)
    delete ik_solver_;
  if(ik_vel_solver_)
    delete ik_vel_solver_;
  if(fk_solver_)
    delete fk_solver_;
}

bool KDLRobotModel::init(std::string robot_description, std::vector<std::string> &planning_joints)
{
  urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
  if (!urdf_->initString(robot_description))
  {
    ROS_ERROR("Failed to parse the URDF.");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*urdf_, ktree_))
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

  /*
  if(!leatherman::getChainTip(ktree_, segments, chain_root_name_, chain_tip_name_))
  {
    ROS_ERROR("Failed to find a valid chain tip link.");
    return false;
  }
  */

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

  // joint name -> index mapping
  for(size_t i = 0; i < planning_joints_.size(); ++i)
    joint_map_[planning_joints_[i]] = i;

  // link name -> kdl index mapping
  for(size_t i = 0; i < kchain_.getNrOfSegments(); ++i)
    link_map_[kchain_.getSegment(i).getName()] = i;

  initialized_ = true;
  return true;
}

bool KDLRobotModel::getJointLimits(std::vector<std::string> &joint_names, std::vector<double> &min_limits, std::vector<double> &max_limits, std::vector<bool> &continuous)
{
  min_limits.resize(joint_names.size());
  max_limits.resize(joint_names.size());
  continuous.resize(joint_names.size());
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    if(joint_names[i].empty())
    {
      ROS_ERROR("Empty joint name found.");
      return false;
    }
    bool c;
    if(!getJointLimits(joint_names[i], min_limits[i], max_limits[i], c))
    {
      ROS_ERROR("Joint limits were not found for %s.", joint_names[i].c_str());
      return false;
    }
    continuous[i] = c;
  }
  return true;
}

bool KDLRobotModel::getJointLimits(std::string joint_name, double &min_limit, double &max_limit, bool &continuous)
{
  bool found_joint = false;
  boost::shared_ptr<const urdf::Link> link = urdf_->getLink(chain_tip_name_);
  while(link && (link->name != chain_root_name_) && !found_joint)
  {
    boost::shared_ptr<const urdf::Joint> joint = urdf_->getJoint(link->parent_joint->name);
    if(joint->name.compare(joint_name) == 0)
    {
      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        if(joint->type != urdf::Joint::CONTINUOUS)
        {
          continuous = false;

          if(joint->safety == NULL)
          {
            min_limit = joint->limits->lower;
            max_limit = joint->limits->upper;
          }
          else
          {
            min_limit = joint->safety->soft_lower_limit;
            max_limit = joint->safety->soft_upper_limit;
          }
        }
        else
        {
          min_limit = -M_PI;
          max_limit = M_PI;
          continuous = true;
        }
      }
      found_joint = true;
    }
    link = urdf_->getLink(link->getParent()->name);
  }
  return found_joint;
}

bool KDLRobotModel::checkJointLimits(const std::vector<double> &angles)
{
  std::vector<double> a = angles;
  if(!sbpl::utils::NormalizeAnglesIntoRange(a, min_limits_, max_limits_))
  {
    ROS_DEBUG("Joint angles are out of bounds.");  
    return false;
  }

  return true;
}

bool KDLRobotModel::computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f)
{
  for(size_t i = 0; i < angles.size(); ++i)
    jnt_pos_in_(i) = angles::normalize_angle(angles[i]);

  KDL::Frame f1;
  if(fk_solver_->JntToCart(jnt_pos_in_, f1, link_map_[name]) < 0)
  {
    ROS_ERROR("JntToCart returned < 0.");
    return false;
  }
  f = T_kinematics_to_planning_ * f1;

  /*
  KDL::Frame f1;
  for(std::map<std::string, int>::const_iterator iter = link_map_.begin(); iter != link_map_.end(); ++iter)
  {
    if(fk_solver_->JntToCart(jnt_pos_in_, f1, iter->second) < 0)
    {
      ROS_ERROR("JntToCart returned < 0.");
      return false;
    }

    f = T_kinematics_to_planning_ * f1;
    leatherman::printKDLFrame(f,iter->first);
  }
  */

  return true;
}

bool KDLRobotModel::computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose)
{
  KDL::Frame f;
  pose.resize(6,0);
  if(computeFK(angles, name, f))
  {
    pose[0] = f.p[0];
    pose[1] = f.p[1];
    pose[2] = f.p[2];
    f.M.GetRPY(pose[3], pose[4], pose[5]);
    return true;
  }
  return false;
}

bool KDLRobotModel::computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose)
{
  KDL::Frame f, f1;
  pose.resize(6,0);
  for(size_t i = 0; i < angles.size(); ++i)
    jnt_pos_in_(i) = angles::normalize_angle(angles[i]);

  if(fk_solver_->JntToCart(jnt_pos_in_, f1, link_map_[planning_link_]) < 0)
  {
    ROS_ERROR("JntToCart returned < 0.");
    return false;
  }

  f = T_kinematics_to_planning_ * f1;

  pose[0] = f.p[0];
  pose[1] = f.p[1];
  pose[2] = f.p[2];
  f.M.GetRPY(pose[3], pose[4], pose[5]);
  return true;
}

bool KDLRobotModel::computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, int option)
{
  if(option == sbpl_arm_planner::ik_option::RESTRICT_XYZ_JOINTS)
    return false;

  return computeIKSearch(pose, start, solution, 0.005);
}

bool KDLRobotModel::computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution)
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

  if(ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = angles::normalize_angle(jnt_pos_out_(i));

  return true;
}

bool KDLRobotModel::computeIKSearch(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, double timeout)
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

  double initial_guess = jnt_pos_in_(free_angle_);
  double search_discretization_angle = 0.02;

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((min_limits_[free_angle_]-initial_guess)/search_discretization_angle);
  int num_negative_increments = (int)((initial_guess-min_limits_[free_angle_])/search_discretization_angle);
  while(loop_time < timeout)
  {
    if(ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) >= 0)
    {
      solution.resize(start.size());
      for(size_t i = 0; i < solution.size(); ++i)
        solution[i] = angles::normalize_angle(jnt_pos_out_(i));
      return true;
    }
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return false;
    jnt_pos_in_(free_angle_) = initial_guess + search_discretization_angle * count;
    ROS_DEBUG("%d, %f",count,jnt_pos_in_(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return false;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return false;
  }
  return false;
}

void KDLRobotModel::printRobotModelInformation()
{
  leatherman::printKDLChain(kchain_, "robot_model");

  ROS_INFO("Joint<->Index Map:");
  for(std::map<std::string, int>::const_iterator iter = joint_map_.begin(); iter != joint_map_.end(); ++iter)
     ROS_INFO("%22s: %d", iter->first.c_str(), iter->second);

  ROS_INFO("Link<->KDL_Index Map:");
  for(std::map<std::string, int>::const_iterator iter = link_map_.begin(); iter != link_map_.end(); ++iter)
     ROS_INFO("%22s: %d", iter->first.c_str(), iter->second);
}


bool KDLRobotModel::getCount(int &count, const int &max_count, const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

}
