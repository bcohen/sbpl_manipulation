#include <sbpl_collision_checking/sbpl_collision_model.h>

namespace sbpl_arm_planner
{

SBPLCollisionModel::SBPLCollisionModel() : ph_("~") 
{
  urdf_.reset();
  dgroup_ = NULL;
}

SBPLCollisionModel::~SBPLCollisionModel()
{
  // delete groups
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); iter++)
  {
    if(iter->second != NULL)
      delete iter->second;
  }
}

bool SBPLCollisionModel::init()
{
  if(!getRobotModel())
    return false;

  return readGroups();
}

bool SBPLCollisionModel::getRobotModel()
{
  std::string robot_description;
  if (nh_.getParam("robot_description", robot_description))
  {
    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!urdf_->initString(robot_description))
    {
      ROS_WARN("Failed to parse the URDF");
      return false;
    }
  }
  else
  {
    ROS_ERROR("The robot description was not found on the param server.");
    return false;
  }

  return true;
}

bool SBPLCollisionModel::readGroups()
{
  XmlRpc::XmlRpcValue all_groups, all_spheres;

  // collision spheres
  std::string spheres_name = "collision_spheres";
  if(!ph_.hasParam(spheres_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << spheres_name);
    return false;
  }
  ph_.getParam(spheres_name, all_spheres);

  if(all_spheres.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    ROS_WARN("Spheres is not an array.");

  if(all_spheres.size() == 0) 
  {
    ROS_WARN("No spheres in spheres");
    return false;
  }

  // collision groups
  std::string group_name = "collision_groups";
  if(!ph_.hasParam(group_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << group_name);
    return false;
  }
  ph_.getParam(group_name, all_groups);

  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    ROS_WARN("Groups is not an array.");

  if(all_groups.size() == 0) 
  {
    ROS_WARN("No groups in groups");
    return false;
  }

  for(int i = 0; i < all_groups.size(); i++) 
  {
    if(!all_groups[i].hasMember("name"))
    {
      ROS_WARN("All groups must have a name.");
      return false;
    }
    std::string gname = all_groups[i]["name"];
    Group* gc = new Group(gname);
    std::map< std::string, Group*>::iterator group_iterator = group_config_map_.find(gname);
    if(group_iterator != group_config_map_.end())
    {
      ROS_WARN_STREAM("Already have group name " << gname);
      delete gc;
      continue;
    }
    group_config_map_[gname] = gc;
    if(!group_config_map_[gname]->getParams(all_groups[i], all_spheres))
    {
      ROS_ERROR("Failed to get all params for %s", gname.c_str());
      return false;
    }
  }
  ROS_INFO("Successfully parsed collision model");
  return true;
}

void SBPLCollisionModel::getGroupNames(std::vector<std::string> &names)
{
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
    names.push_back(iter->first);
}

void SBPLCollisionModel::setDefaultGroup(std::string group_name)
{
  dgroup_ = group_config_map_[group_name];
}

void SBPLCollisionModel::printGroups()
{
  if(group_config_map_.begin() == group_config_map_.end())
  {
    ROS_ERROR("No groups found.");
    return;
  }

  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    if(!iter->second->init_)
    {
      ROS_ERROR("Failed to print %s group information because has not yet been initialized.", iter->second->getName().c_str());
      continue;
    }
    iter->second->print();
    ROS_INFO("----------------------------------");
  }
}

bool SBPLCollisionModel::getFrameInfo(std::string &name, std::string group_name, int &chain, int &segment)
{
  return group_config_map_[group_name]->getFrameInfo(name, chain, segment);
}

bool SBPLCollisionModel::initAllGroups()
{
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    if(!iter->second->init(urdf_))
      return false;
  }
  return true;
}

bool SBPLCollisionModel::computeDefaultGroupFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame> > &frames)
{
  return computeGroupFK(angles, dgroup_, frames);
}

bool SBPLCollisionModel::computeGroupFK(const std::vector<double> &angles, Group* group, std::vector<std::vector<KDL::Frame> > &frames)
{
  return group->computeFK(angles, frames);
}

void SBPLCollisionModel::setOrderOfJointPositions(const std::vector<std::string> &joint_names, std::string group_name)
{
  group_config_map_[group_name]->setOrderOfJointPositions(joint_names);
}

void SBPLCollisionModel::setJointPosition(const std::string &name, double position)
{
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); iter++)
    iter->second->setJointPosition(name, position);
}

void SBPLCollisionModel::printDebugInfo(std::string group_name)
{
  Group* group = group_config_map_[group_name];
  group->printDebugInfo();
}

void SBPLCollisionModel::getDefaultGroupSpheres(std::vector<Sphere *> &spheres)
{
  dgroup_->getSpheres(spheres);
}

bool SBPLCollisionModel::getJointLimits(std::string group_name, std::string joint_name, double &min_limit, double &max_limit, bool &continuous)
{
  if(group_config_map_.find(group_name) == group_config_map_.end())
    return false;
  if(!group_config_map_[group_name]->init_)
    return false;

  return leatherman::getJointLimits(urdf_.get(), group_config_map_[group_name]->getReferenceFrame(), group_config_map_[group_name]->tip_name_, joint_name, min_limit, max_limit, continuous);
}

std::string SBPLCollisionModel::getReferenceFrame(std::string group_name)
{
  if(group_config_map_.find(group_name) == group_config_map_.end())
    return "";
  if(!group_config_map_[group_name]->init_)
    return "";
  return group_config_map_[group_name]->getReferenceFrame();
}

Group* SBPLCollisionModel::getGroup(std::string name)
{
  Group* r = NULL;
  if(group_config_map_.find(name) == group_config_map_.end())
    return r;
  return group_config_map_[name];
}

void SBPLCollisionModel::getVoxelGroups(std::vector<Group*> &vg)
{
  vg.clear();
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    if(iter->second->type_ == sbpl_arm_planner::Group::VOXELS)
      vg.push_back(iter->second);
  }
}

bool SBPLCollisionModel::doesLinkExist(std::string name, std::string group_name)
{ 
  int chain, segment;
  return getFrameInfo(name, group_name, chain, segment);
}

bool SBPLCollisionModel::setModelToWorldTransform(const arm_navigation_msgs::MultiDOFJointState &state, std::string world_frame)
{
  KDL::Frame f;

  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    if(world_frame.compare(iter->second->getReferenceFrame()) != 0)
    {
      if(!leatherman::getFrame(state, world_frame, iter->second->getReferenceFrame(), f))
      { 
        ROS_ERROR("Failed to get transform from world frame, '%s', to the reference frame, '%s' for collision group, '%s'.", world_frame.c_str(), iter->second->getReferenceFrame().c_str(), iter->second->getName().c_str());
        return false;
      }
      else
        iter->second->setGroupToWorldTransform(f);
    }
    else
    {
      f = KDL::Frame::Identity();
      iter->second->setGroupToWorldTransform(f);
    }
  }
  return true;
}

}
