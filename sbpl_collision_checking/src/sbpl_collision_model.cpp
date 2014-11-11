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

bool SBPLCollisionModel::init(std::string ns)
{
  if(!getRobotModel())
    return false;

  if(ns.empty())
    ns = "~";
  
  if(!readGroups(ns))
    return false;

  return readCollisionMatrix(ns);
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

bool SBPLCollisionModel::readGroups(std::string ns)
{
  XmlRpc::XmlRpcValue all_groups, all_spheres;
  ros::NodeHandle nh(ns);

  // collision spheres
  std::string spheres_name = "collision_spheres";
  if(!nh.hasParam(spheres_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << spheres_name);
    return false;
  }
  nh.getParam(spheres_name, all_spheres);

  if(all_spheres.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    ROS_WARN("Spheres is not an array.");

  if(all_spheres.size() == 0) 
  {
    ROS_WARN("No spheres in spheres");
    return false;
  }

  // collision groups
  std::string group_name = "collision_groups";
  if(!nh.hasParam(group_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << group_name);
    return false;
  }
  nh.getParam(group_name, all_groups);

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

  int id = 0;
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    for(int l = 0; l < int(iter->second->links_.size()); ++l)
    {
      iter->second->links_[l].setLinkID(id);
      if((link_map_.find(iter->second->links_[l].name_) != link_map_.end()) && !link_map_.empty())
      {
        ROS_ERROR("Found duplicate link name in the collision model. All link names have to be unique. Exiting. (%s)", iter->second->links_[l].name_.c_str());
        return false;
      }
      link_map_[iter->second->links_[l].name_] = &(iter->second->links_[l]);
      id++;
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

bool SBPLCollisionModel::setDefaultGroup(std::string group_name)
{
  if(group_config_map_.find(group_name) == group_config_map_.end())
    return false;

  dgroup_ = group_config_map_[group_name];

  // initialize set of sphere groups (for fast reference by the collision space)
  sphere_groups_.clear();
  sphere_groups_.push_back(dgroup_);
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    if((iter->second->type_ == sbpl_arm_planner::Group::SPHERES) && 
       (iter->second->getName().compare(group_name) != 0))
      sphere_groups_.push_back(iter->second);
  }
  return true;
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

void SBPLCollisionModel::getDefaultGroupSpheres(std::vector<Sphere *> &spheres, bool low_res)
{
  dgroup_->getSpheres(spheres, low_res);
}

std::vector<Sphere*> SBPLCollisionModel::getGroupSpheres(std::string group_name, bool low_res)
{
  std::vector<Sphere*> spheres;
  group_config_map_[group_name]->getSpheres(spheres, low_res);
  return spheres;
}

bool SBPLCollisionModel::getJointLimits(std::string group_name, std::string joint_name, double &min_limit, double &max_limit, bool &continuous)
{
  if(group_config_map_.find(group_name) == group_config_map_.end())
    return false;
  if(!group_config_map_[group_name]->init_)
    return false;

  ROS_DEBUG("root name: %s  tip name: %s  joint name: %s", group_config_map_[group_name]->getReferenceFrame().c_str(), group_config_map_[group_name]->tip_name_.c_str(), joint_name.c_str());
  return leatherman::getJointLimits(urdf_.get(), group_config_map_[group_name]->getReferenceFrame(), group_config_map_[group_name]->tip_name_, joint_name, min_limit, max_limit, continuous, false);
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

void SBPLCollisionModel::getSphereGroups(std::vector<Group*> &vg)
{ 
  if(!sphere_groups_.empty())
    vg = sphere_groups_;
  else
  {
    vg.clear();
    for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
    {
      if(iter->second->type_ == sbpl_arm_planner::Group::SPHERES)
        vg.push_back(iter->second);
    }
  }
}

void SBPLCollisionModel::setSphereGroupsForCheckCollision(const std::vector<std::string> &group_names)
{
  sphere_groups_.clear();
  sphere_groups_.push_back(dgroup_);
  for(size_t i = 0; i < group_names.size(); ++i)
  {
    if(group_config_map_.find(group_names[i]) != group_config_map_.end() && 
        (dgroup_->getName().compare(group_names[i]) != 0))
    {
      sphere_groups_.push_back(group_config_map_[group_names[i]]);
    }
  }
  ROS_INFO("Checking group '%s' against %d other collision groups.", dgroup_->getName().c_str(), int(sphere_groups_.size()-1));
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
      {
        iter->second->setGroupToWorldTransform(f);
        leatherman::printKDLFrame(f, "group-world");
      }
    }
    else
    {
      f = KDL::Frame::Identity();
      iter->second->setGroupToWorldTransform(f);
    }
  }
  return true;
}

bool SBPLCollisionModel::readCollisionMatrix(std::string ns)
{
  XmlRpc::XmlRpcValue all_ops;
  ros::NodeHandle nh(ns);

  // initialize collision matrix to true
  collision_matrix_.resize(link_map_.size(), std::vector<bool> (link_map_.size(), true));

  for(size_t i = 0; i < collision_matrix_.size(); ++i)
    ROS_INFO("[%2d] check: %s", int(i), leatherman::getString(collision_matrix_[i], "yes", "no ").c_str());

  // set each link to not check itself
  for(size_t j = 0; j < collision_matrix_.size(); ++j)
    collision_matrix_[j][j] = false;

  // pull collision operation off param server
  std::string matrix_name = "default_collision_operations";
  if(!nh.hasParam(matrix_name)) 
  {
    ROS_WARN("No collision matrix found on param server for '%s'. ", matrix_name.c_str());
    return false;
  }
  nh.getParam(matrix_name, all_ops);

  if(all_ops.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    ROS_WARN("The collision operations found on the param server is not a list.");

  if(all_ops.size() == 0) 
  {
    ROS_WARN("No collision operations found in the list.");
    return false;
  }

  ROS_INFO("hey");
  for(int i = 0; i < all_ops.size(); i++) 
  {
    if(!all_ops[i].hasMember("object1"))
    {
      ROS_WARN("All groups must have a name.");
      return false;
    }
    if(!all_ops[i].hasMember("object2"))
    {
      ROS_WARN("All groups must have a name.");
      return false;
    }
    if(!all_ops[i].hasMember("operation"))
    {
      ROS_WARN("All collision operation pairs must have an operation. Duh.");
      return false;
    }

    std::string object1 = all_ops[i]["object1"];
    std::string object2 = all_ops[i]["object2"];
    std::string op_value = all_ops[i]["operation"];

    if(link_map_.find(object1) == link_map_.end() || link_map_.find(object2) == link_map_.end())
    {
      ROS_ERROR("Either object1(%s) or object2(%s) is not found in the entire list of links. Exiting.", object1.c_str(), object2.c_str());
      return false;
    }

    ROS_INFO("object1: %s %d, object2: %s %d", object1.c_str(), link_map_[object1]->id_, object2.c_str(), link_map_[object2]->id_);

    if(op_value.compare("disable") == 0)
    {
      collision_matrix_[link_map_[object1]->id_][link_map_[object2]->id_] = false;
      collision_matrix_[link_map_[object2]->id_][link_map_[object1]->id_] = false;
    }
  }

  for(size_t i = 0; i < collision_matrix_.size(); ++i)
    ROS_INFO("[%2d] check: %s", int(i), leatherman::getString(collision_matrix_[i], "yes", "no ").c_str());

  ROS_INFO("Successfully parsed collision matrix");
  return true;
}

bool SBPLCollisionModel::doLinkToLinkCheck(int link1, int link2)
{
  if(link1 > int(collision_matrix_.size()) || link2 > int(collision_matrix_.size()))
  {
    ROS_ERROR("Received invalid link_id's. There might be a problem.");
    return false;
  }

  return collision_matrix_[link1][link2];
}

}
