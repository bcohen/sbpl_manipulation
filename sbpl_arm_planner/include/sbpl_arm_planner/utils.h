#ifndef _SBPL_ARM_PLANNER_UTILS_
#define _SBPL_ARM_PLANNER_UTILS_

#include <iostream>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
//#include <planning_scene/planning_scene.h>
//#include <planning_models/transforms.h>
//#include <planning_models/kinematic_model.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/Pose.h>


namespace sbpl_arm_planner{

void setRPY(double roll, double pitch, double yaw, Eigen::Matrix3d &m);
void getRPY(const Eigen::Matrix3d &m, double &roll, double &pitch, double &yaw);
void getRPY(const geometry_msgs::Quaternion &qmsg, double &roll, double &pitch, double &yaw);
//void transformPose(const planning_scene::PlanningSceneConstPtr pscene, geometry_msgs::Pose &min, geometry_msgs::Pose &mout, std::string from, std::string to);
void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e);
void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k);

void printAffine3d(const Eigen::Affine3d &a, std::string text);
void printPoseMsg(const geometry_msgs::Pose &p, std::string text);
void printKDLFrame(const KDL::Frame &f, std::string text);
}

#endif
