#include <sbpl_arm_planner/collision_checker.h>

CollisionChecker::CollisionChecker(std::string group_name) : group_name_(group_name)
{
  //kinematic_model_loader_.reset(new planning_models_loader::KinematicModelLoader("robot_description"));

  // size of increments for when doing path interpolation
  inc_.resize(7,0.0348);
  inc_[6] = 0.13963634;

  ROS_ERROR("[cc] Joint limits are hardcoded in for the RIGHT arm. Will fix after IROS.");
  min_limits_.resize(7, 0.0);
  max_limits_.resize(7, 0.0);

  min_limits_[0] = -2.29; min_limits_[1] = -0.524; min_limits_[2] = -3.910;
  min_limits_[3] = -2.29; min_limits_[4] = -M_PI; min_limits_[5] = -2.2; min_limits_[6] = -M_PI;
  max_limits_[0] = 0.715; max_limits_[1] = 1.396; max_limits_[2] = 0.865;
  max_limits_[3] = 0.300; max_limits_[4] = M_PI; max_limits_[5] = 0.1; max_limits_[6] = M_PI;

  ROS_INFO("[cc] Collision checker created.");
}

bool CollisionChecker::setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene)
{
  ROS_INFO("[cc] Setting planning scene...");
  planning_scene_ = planning_scene;
  ROS_INFO("[cc] planning scene name: %s", planning_scene->getName().c_str());
  kstate_ = new planning_models::KinematicState(planning_scene->getCurrentState());
  jsg_ = kstate_->getJointStateGroup(group_name_);

  std::vector<double> ja;
  std::vector<std::string> jn = jsg_->getJointNames();
  jsg_->getGroupStateValues(ja);
  ROS_INFO("[cc] ja size: %d  jn size: %d", int(ja.size()), int(jn.size()));
  for(size_t i = 0; i < ja.size(); ++i)
    ROS_INFO("[cc] [%d] %s: %0.3f", int(i), jn[i].c_str(), ja[i]);


  hy_world_ = dynamic_cast<const collision_detection::CollisionWorldHybrid*>(planning_scene->getCollisionWorld().get());
  if(!hy_world_) 
  {
    ROS_WARN("[cc] Could not initialize hybrid collision world from planning scene");
    return false;
  }

  hy_robot_ = dynamic_cast<const collision_detection::CollisionRobotHybridROS*>(planning_scene->getCollisionRobot().get());
  if(!hy_robot_) 
  {
    ROS_WARN("[cc] Could not initialize hybrid collision robot from planning scene");
    return false;
  }

  req_.group_name = group_name_;
  req_.contacts = false;
  req_.distance = true;
  first_run_ = true;

  ROS_INFO("[cc] The planning scene has been set.");
  return true;
}

bool CollisionChecker::checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, unsigned char &dist)
{
  // assume the joint mapping is shoulder_pan -> wrist_roll
  if(!jsg_->setStateValues(angles))
    return false;
/*
  if(first_run_)
  {
    first_run_ = false;
    hy_world_->checkCollision(req_, res_,*hy_robot_, *kstate_, planning_scene_->getAllowedCollisionMatrix(), gsr_);
    hy_world_->checkCollision(req_, res_,*hy_robot_, *kstate_, planning_scene_->getAllowedCollisionMatrix(), gsr_);
    if(res_.collision)
      return false;
  }
*/

  hy_world_->checkCollisionDistanceField(req_, res_, *hy_robot_->getCollisionRobotDistanceField().get(), *kstate_, planning_scene_->getAllowedCollisionMatrix(), gsr_);
  //hy_world_->checkCollision(req_, res_,*hy_robot_, *kstate_, gsr_);

  if(res_.collision)
    return false;

  return true;
}

bool CollisionChecker::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, int &path_length, int &num_checks, unsigned char &dist)
{
  int inc_cc = 5;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  dist = 100;
  num_checks = 0;

  for(size_t i=0; i < start.size(); ++i)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }

  if(!sbpl::Interpolator::interpolatePath(start_norm, end_norm, min_limits_, max_limits_, inc_, path))
  {
    path_length = 0;
    ROS_ERROR("[cc] Failed to interpolate the path. It's probably infeasible due to joint limits.");
    return false;
  }

  // for debugging & statistical purposes
  path_length = path.size();

  // try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        num_checks++;
        if(!checkCollision(path[j], verbose, false, dist_temp))
        {
          dist = dist_temp;
          return false;
        }
        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      num_checks++;
      if(!checkCollision(path[i], verbose, false, dist_temp))
      {
        dist = dist_temp;
        return false;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }
  }
  return true;
}

bool CollisionChecker::interpolatePath(const std::vector<double>& start,
                                         const std::vector<double>& end,
                                         const std::vector<double>& inc,
                                         std::vector<std::vector<double> >& path)
{
  std::vector<double> min_limits, max_limits;

  // for the right arm!
  ROS_ERROR("[cc] Joint limits are hardcoded in for the RIGHT arm. Will fix after IROS.");
  min_limits.resize(7, 0.0);
  max_limits.resize(7, 0.0);

  min_limits[0] = -2.135398; min_limits[1] = -0.3536; min_limits[2] = -3.75;
  min_limits[3] = -2.1213; min_limits[4] = -M_PI; min_limits[5] = -2.0; min_limits[6] = -M_PI;
  max_limits[0] = 0.715; max_limits[1] = 1.396; max_limits[2] = 0.865;
  max_limits[3] = 0.150; max_limits[4] = M_PI; max_limits[5] = 0.0; max_limits[6] = M_PI;

  return sbpl::Interpolator::interpolatePath(start, end, min_limits, max_limits, inc, path);
}


