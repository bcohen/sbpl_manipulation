#ifndef _POST_PROCESSING_
#define _POST_PROCESSING_

#include <vector>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sbpl_manipulation_components/collision_checker.h>

namespace sbpl_arm_planner {

  void shortcutPath(sbpl_arm_planner::CollisionChecker *cc, std::vector<std::vector<double> > &pin, std::vector<std::vector<double> > &pout);

  void shortcutTrajectory(sbpl_arm_planner::CollisionChecker *cc, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_in, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_out);

  bool interpolateTrajectory(sbpl_arm_planner::CollisionChecker *cc, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_out);

};

#endif
