#include <sbpl_arm_planner/utils.h>

void sbpl_arm_planner::setRPY(double roll, double pitch, double yaw, Eigen::Matrix3d &m)
{
  m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void sbpl_arm_planner::getRPY(const Eigen::Matrix3d &m, double &roll, double &pitch, double &yaw)
{
  Eigen::Vector3d v = m.eulerAngles(0,1,2);
  roll = v(0);  pitch = v(1);  yaw = v(2);
}

void sbpl_arm_planner::getRPY(const geometry_msgs::Quaternion &qmsg, double &roll, double &pitch, double &yaw)
{
  geometry_msgs::Pose pose;
  pose.orientation = qmsg;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  ROS_DEBUG("[utils] rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f\n", roll, pitch, yaw, qmsg.x, qmsg.y, qmsg.z, qmsg.w);fflush(stdout);
}

void sbpl_arm_planner::printAffine3d(const Eigen::Affine3d &a, std::string text)
{
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f", text.c_str(), a.translation().x(), a.translation().y(), a.translation().z());
}

void sbpl_arm_planner::printPoseMsg(const geometry_msgs::Pose &p, std::string text)
{
  double r,pitch,y; getRPY(p.orientation,r,pitch,y);
  printf("[%s] xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f\n", text.c_str(), p.position.x, p.position.y, p.position.z, r, pitch, y, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);fflush(stdout);
}

void sbpl_arm_planner::printKDLFrame(const KDL::Frame &f, std::string text)
{
  double r,p,y; f.M.GetRPY(r,p,y);
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", text.c_str(), f.p[0], f.p[1], f.p[2], r, p, y);
}

void sbpl_arm_planner::transformPose(const planning_scene::PlanningSceneConstPtr pscene, geometry_msgs::Pose &min, geometry_msgs::Pose &mout, std::string from, std::string to)
{
  Eigen::Affine3d pin, pout, tmp;
  planning_models::poseFromMsg(min,pin);
  pscene->getTransforms()->transformPose(pscene->getCurrentState(), from, pin, tmp);
  pout = tmp * pscene->getTransforms()->getTransform(pscene->getCurrentState(),to).inverse();
  planning_models::msgFromPose(pout, mout);
}

void sbpl_arm_planner::transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e)
{
  e(0,3) = k.p[0];
  e(1,3) = k.p[1];
  e(2,3) = k.p[2];

  e(0,0) = k.M(0,0);
  e(0,1) = k.M(0,1);
  e(0,2) = k.M(0,2);
  e(1,0) = k.M(1,0);
  e(1,1) = k.M(1,1);
  e(1,2) = k.M(1,2);
  e(2,0) = k.M(2,0);
  e(2,1) = k.M(2,1);
  e(2,2) = k.M(2,2);

  e(3,0) = 0.0;
  e(3,1) = 0.0;
  e(3,2) = 0.0;
  e(3,3) = 1.0;
}

void sbpl_arm_planner::transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  k.p[0] = e(0,3);
  k.p[1] = e(1,3);
  k.p[2] = e(2,3);

  k.M(0,0) = e(0,0);
  k.M(0,1) = e(0,1);
  k.M(0,2) = e(0,2);
  k.M(1,0) = e(1,0);
  k.M(1,1) = e(1,1);
  k.M(1,2) = e(1,2);
  k.M(2,0) = e(2,0);
  k.M(2,1) = e(2,1);
  k.M(2,2) = e(2,2);
}

