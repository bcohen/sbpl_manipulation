#include <ros/console.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_manipulation_components/post_processing.h>

void sbpl_arm_planner::shortcutPath(sbpl_arm_planner::CollisionChecker *cc, std::vector<std::vector<double> > &pin, std::vector<std::vector<double> > &pout)
{
  int i = 0;
  int num_checks=0, path_length=0;
  unsigned int current=1;
  double dist=100;

  if(pin.size() <= 2)
  {
    ROS_INFO("Path has 2 waypoints or less. Can't shortcut.");
    pout = pin;
    return;
  }

  pout.clear();
  pout.push_back(pin[0]);
  ROS_DEBUG("[ keeping %d] %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",0,pin[0][0],pin[0][1],pin[0][2],pin[0][3],pin[0][4],pin[0][5],pin[0][6]);
  while(current != pin.size())
  {
    if(!cc->isStateToStateValid(pin[i], pin[current], path_length, num_checks, dist))
    {
      i = current;
      ROS_DEBUG("[ keeping %d] %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f (num_checks: %d  interp_length: %d)",current,pin[i][0],pin[i][1],pin[i][2],pin[i][3],pin[i][4],pin[i][5],pin[i][6], num_checks, path_length);
      pout.push_back(pin[i]);
    }
    else
    {
      if(current < pin.size()-1)
        ROS_DEBUG("[ cutting %d] %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f (num_checks: %d  interp_length: %d)",current,pin[current][0],pin[current][1],pin[current][2],pin[current][3],pin[current][4],pin[current][5],pin[current][6], num_checks, path_length);
      else
      {
        ROS_DEBUG("[ keeping %d] %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f (num_checks: %d  interp_length: %d)",int(pin.size())-1,pin[pin.size()-1][0],pin[pin.size()-1][1],pin[pin.size()-1][2],pin[pin.size()-1][3],pin[pin.size()-1][4],pin[pin.size()-1][5],pin[pin.size()-1][6], num_checks, path_length);
        pout.push_back(pin[current]);
      }
    }
    current++;
  }
  if(pout.size() == 2)
    ROS_ERROR("Does it make sense that the shortcutted path length is 2? Is it freespace?");

  ROS_INFO("Original path length: %d   Shortcutted path length: %d", int(pin.size()), int(pout.size()));
}

void sbpl_arm_planner::shortcutTrajectory(sbpl_arm_planner::CollisionChecker *cc, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_in, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_out)
{
  std::vector<std::vector<double> > pin(traj_in.size()), pout;

  for(size_t j = 0; j < traj_in.size(); ++j)
  {
    pin[j].resize(traj_in[j].positions.size(),0);
    for(size_t k = 0; k < traj_in[j].positions.size(); ++k)
      pin[j][k] = traj_in[j].positions[k];
  }

  if(pin.size() > 2)
    sbpl_arm_planner::shortcutPath(cc, pin, pout);
  else
    ROS_WARN("Path is too short for shortcutting.");

  traj_out.resize(pout.size());
  for(size_t j = 0; j < pout.size(); ++j)
  {
    for(size_t k = 0; k < pout[j].size(); ++k)
    {
      traj_out[j].positions.resize(pout[j].size());
      traj_out[j].positions[k] = pout[j][k];
    }
  }
}

bool sbpl_arm_planner::interpolateTrajectory(sbpl_arm_planner::CollisionChecker *cc, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj, std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_out)
{
  if(traj.empty())
    return false;
 
  int num_joints = traj[0].positions.size();
  std::vector<std::vector<double> > path, ipath;
  std::vector<double> start(num_joints,0), end(num_joints,0), inc(num_joints,0.069);

  // tack on the first point of the trajectory
  for(size_t j = 0; j < traj[0].positions.size(); ++j)
    start[j] = traj[0].positions[j];
  path.push_back(start);

  for(size_t i = 0; i < traj.size()-1; ++i)
  {
    for(size_t j = 0; j < traj[i].positions.size(); ++j)
    {
      start[j] = traj[i].positions[j];
      end[j] = traj[i+1].positions[j];
    }
    ipath.clear();

    if(!cc->interpolatePath(start, end, inc, ipath))
    {
      ROS_ERROR("Failed to interpolate between waypoint %d & %d because it's infeasible given the limits.", int(i), int(i+1));
      return false;
    }

    if(ipath.empty())
    {
      ROS_ERROR("Interpolated path between waypoints %d & %d is empty...what's going on?", int(i), int(i+1));
      return false;
    }

    // we already have the first waypoint in the path (from last iteration)
    ipath.erase(ipath.begin());

    // concatenate current path and the intermediate path
    path.insert(path.end(),ipath.begin(),ipath.end());

    ROS_DEBUG("[%d] path length: %d", int(i), int(path.size()));
  }

  traj_out.resize(path.size());
  for(size_t i = 0; i < path.size(); ++i)
  {
    traj_out[i].positions.resize(path[i].size());
    for(size_t j = 0; j < path[i].size(); ++j)
      traj_out[i].positions[j] = path[i][j];
    traj_out[i].time_from_start.fromSec(double(i+1) * (traj.back().time_from_start.toSec()/double(path.size())));
  }
  ROS_INFO("Original path length: %d   Interpolated path length: %d", int(traj.size()), int(traj_out.size()));
  return true;
}

