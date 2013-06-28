#include <sbpl_collision_checking/sbpl_collision_model.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <geometric_shapes/shapes.h>
#include <leatherman/utils.h>

#define RESOLUTION 0.02

namespace sbpl_arm_planner
{

bool sortSphere(sbpl_arm_planner::Sphere* a, sbpl_arm_planner::Sphere* b)
{
  return a->priority < b->priority;
}

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

  readGroups();

  return true;
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

/*
bool SBPLCollisionModel::initKDLChainForGroup(std::string &chain_root_name, Group* group)
{
  bool valid_tip = false;

  if (!kdl_parser::treeFromUrdfModel(*urdf_, kdl_tree_))
  {
    ROS_ERROR("Failed to parse tree from robot description.");
    return false;
  }

  for(size_t i = 0; i < group->links_.size(); ++i)
  {
    if(valid_tip)
      break;

    if (!kdl_tree_.getChain(chain_root_name, group->links_[i].root_name_, group->chain_))
    {
      ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", chain_root_name.c_str(), group->links_[i].root_name_.c_str());
      return false;
    }

    int found = 0;
    for(size_t j = 0; j < group->links_.size(); ++j)
    {
      for(size_t k = 0; k < group->chain_.getNrOfSegments(); ++k)
      {
        if(group->chain_.getSegment(k).getName().compare(group->links_[j].root_name_) == 0)
        {
          found++;
          break;
        }
      }
      if(found == int(group->links_.size()))
      {
        ROS_INFO("%s is a good tip link for the %s chain", group->links_[i].root_name_.c_str(),group->name_.c_str());
        valid_tip = true;
        break;
      }
      else
        ROS_DEBUG("%s is NOT a good tip link for the %s chain", group->links_[i].root_name_.c_str(), group->name_.c_str());
    }
  }
 
  if(!valid_tip)
  {
    ROS_ERROR("Unable to initialize a chain that includes all of the joints.");
    return false;
  }
  ROS_INFO("A chain was initialized for the %s from %s to %s.", group->name_.c_str(), chain_root_name.c_str(), group->chain_.getSegment(group->chain_.getNrOfSegments()-1).getName().c_str());
  return true;
}
*/

bool SBPLCollisionModel::initKDLChainForGroup(Group* group)
{
  bool unincluded_links = true;
  int cnt = 0;
  std::vector<int> link_included(group->links_.size(),-1);
  KDL::Chain chain;

  if (!kdl_parser::treeFromUrdfModel(*urdf_, kdl_tree_))
  {
    ROS_ERROR("Failed to parse tree from robot description.");
    return false;
  }

  // loop until all links are included in a single kdl chain
  while(unincluded_links && cnt < 100)
  {
    cnt++;

    std::vector<int> num_links_per_tip(group->links_.size(),0);

    // compute # of links each link would include if tip of chain
    for(size_t i = 0; i < group->links_.size(); ++i)
    {
      // link i is already included in a chain
      if(link_included[i] > -1)
        continue;

      // create chain with link i as the tip
      if (!kdl_tree_.getChain(group->root_name_, group->links_[i].root_name_, chain))
      {
        ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", group->root_name_.c_str(), group->links_[i].root_name_.c_str());
        continue;
      }

      // count number of links included in this chain
      for(size_t j = 0; j < group->links_.size(); ++j)
      {
        // link j is already included in a chain
        if(link_included[j] > -1)
          continue;

        // check if link is included in this chain
        //if(getSegmentNumber(group->links_[j].root_name_, chain) > -1)
        int seg;
        if(leatherman::getSegmentIndex(chain, group->links_[j].root_name_, seg))
          num_links_per_tip[i]++;
      }
    }

    // get chain tip that would include the most links
    int i_max = 0;
    for(size_t i = 0; i < num_links_per_tip.size(); ++i)
    {
      ROS_ERROR("chain_tip: %25s  included_links %d", group->links_[i].root_name_.c_str(), num_links_per_tip[i]);
      if(num_links_per_tip[i] > num_links_per_tip[i_max])
        i_max = i;
    }
    ROS_INFO("Creating a chain for %s group with %s as the tip.", group->name_.c_str(), group->links_[i_max].root_name_.c_str());

    // create chain with link i_max as the tip
    if (!kdl_tree_.getChain(group->root_name_, group->links_[i_max].root_name_, chain))
    {
      ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", group->root_name_.c_str(), group->links_[i_max].root_name_.c_str());
      continue;
    }

    // add chain to the group
    group->chains_.push_back(chain);

    // mark links that are included in this chain
    int included_links = 0;
    for(size_t i = 0; i < group->links_.size(); ++i)
    {
      // link i is already included in a different chain
      if(link_included[i] > -1)
      {
        included_links++;
        continue;
      }

      // check if link i is included in this chain
      //if(getSegmentNumber(group->links_[i].root_name_, group->chains_.back()) > -1)
      int seg;
      if(leatherman::getSegmentIndex(group->chains_.back(), group->links_[i].root_name_, seg))
      {
        link_included[i] = group->chains_.size()-1;
        group->links_[i].i_chain_ = group->chains_.size()-1;
        included_links++;
        ROS_ERROR("[chain: %s] [%d] includes: %s", group->links_[i_max].root_name_.c_str(), included_links, group->links_[i].root_name_.c_str());
      }
    }

    if(included_links == int(group->links_.size()))
      unincluded_links = false;

    ROS_ERROR("Completed %d loops of the while loop (included_links = %d)", cnt, included_links);
  }

  for(size_t i = 0; i < link_included.size(); ++i)
    ROS_INFO("included link: %25s  link_root: %25s  chain: %d", group->links_[i].name_.c_str(), group->links_[i].root_name_.c_str(), link_included[i]);

  if(cnt >= 100)
    return false;

  // initialize the FK solvers
  initFKSolvers(group);

  ROS_INFO("Initialized %d chains for the %s group.", int(group->chains_.size()), group->name_.c_str());
  return true;
}

bool SBPLCollisionModel::initGroup(std::string group_name)
{
  if(group_config_map_.find(group_name) == group_config_map_.end())
  {
    ROS_ERROR("Parameters for group '%s' cannot be found. Exiting.", group_name.c_str());
    return false;
  }

  if(!initKDLChainForGroup(group_config_map_[group_name]))
    return false;

  if(!initGroup(group_config_map_[group_name]))
    return false;

  group_config_map_[group_name]->init_ = true;
  return true;
}

void SBPLCollisionModel::readGroups()
{
  XmlRpc::XmlRpcValue all_groups;

  std::string group_name = "collision_groups";

  if(!ph_.hasParam(group_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << group_name);
    return;
  }
  ph_.getParam(group_name, all_groups);

  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    ROS_WARN("Groups is not an array.");

  if(all_groups.size() == 0) 
  {
    ROS_WARN("No groups in groups");
    return;
  }

  for(int i = 0; i < all_groups.size(); i++) 
  {
    if(!all_groups[i].hasMember("name"))
    {
      ROS_WARN("All groups must have a name.");
      continue;
    }
    std::string gname = all_groups[i]["name"];
    Group* gc = new Group(gname);
    gc->init_ = false;
    std::map< std::string, Group*>::iterator group_iterator = group_config_map_.find(gname);
    if(group_iterator != group_config_map_.end())
    {
      ROS_WARN_STREAM("Already have group name " << gname);
      delete gc;
      continue;
    }
    group_config_map_[gname] = gc;

    if(!all_groups[i].hasMember("type"))
    {
      ROS_WARN("All groups must have a type. (voxels or spheres)");
      continue;
    }
    if(std::string(all_groups[i]["type"]).compare("voxels") == 0)
      group_config_map_[gname]->type_ = sbpl_arm_planner::Group::VOXELS;
    else if(std::string(all_groups[i]["type"]).compare("spheres") == 0)
      group_config_map_[gname]->type_ = sbpl_arm_planner::Group::SPHERES;
    else
    {
      ROS_ERROR("Illegal group type. (voxels or spheres)");
      continue;
    }

    if(!all_groups[i].hasMember("root_name"))
    {
      ROS_WARN("All groups must have a root_name.");
      continue;
    }
    group_config_map_[gname]->root_name_ = std::string(all_groups[i]["root_name"]);

    if(!all_groups[i].hasMember("tip_name"))
    {
      ROS_WARN("All groups must have a tip_name.");
      continue;
    }
    group_config_map_[gname]->tip_name_ = std::string(all_groups[i]["tip_name"]);

    /*
    if(!all_groups[i].hasMember("redundancy"))
      ROS_WARN("No redundant joint name set. That's OK. This is just a warning.");
    else
      group_config_map_[gname]->redundancy_name_ = std::string(all_groups[i]["redundancy"]["name"]);

    if(all_groups[i].hasMember("planning_joints"))
    {
      std::string joint_list = std::string(all_groups[i]["planning_joints"]);
      std::stringstream joint_name_stream(joint_list);
      while(joint_name_stream.good() && !joint_name_stream.eof())
      {
        std::string jname;
        joint_name_stream >> jname;
        if(jname.size() == 0) 
          continue;
        if (urdf_->getJoint(jname))
          group_config_map_[gname]->joint_names_.push_back(jname); 
        else 
          ROS_DEBUG_STREAM("Urdf doesn't have joint " << jname);
      }
    }
    */

    if(all_groups[i].hasMember("collision_links"))
    {
      XmlRpc::XmlRpcValue all_links = all_groups[i]["collision_links"];

      if(all_links.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("Collision links is not an array.");
        return;
      }

      if(all_links.size() == 0) 
      {
        ROS_WARN("No links in collision links");
        return;
      }

      Link link;
      for(int j = 0; j < all_links.size(); j++) 
      {
        link.spheres_.clear();
        if(all_links[j].hasMember("name"))
          link.name_ = std::string(all_links[j]["name"]);
        if(all_links[j].hasMember("root"))
          link.root_name_ = std::string(all_links[j]["root"]);
        else
          ROS_WARN("No root name");

        if(group_config_map_[gname]->type_ == sbpl_arm_planner::Group::SPHERES)
        {
          std::stringstream ss(all_links[j]["spheres"]);
          Sphere sphere;
          double x,y,z;
          std::vector<Sphere> link_spheres;
          std::string sphere_name;
          while(ss >> sphere_name)
          {
            ros::NodeHandle sphere_node(ph_, sphere_name);
            sphere.name = sphere_name;
            sphere_node.param("x", x, 0.0);
            sphere_node.param("y", y, 0.0);
            sphere_node.param("z", z, 0.0);
            sphere_node.param("radius", sphere.radius, 0.0);
            sphere_node.param("priority", sphere.priority, 1);
            sphere.v.x(x);
            sphere.v.y(y);
            sphere.v.z(z);
            link.spheres_.push_back(sphere);
          }
        }
        group_config_map_[gname]->links_.push_back(link);
      }
    }
  }

  ROS_INFO("Successfully parsed collision model");
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
      ROS_ERROR("Failed to print %s group information because has not yet been initialized.", iter->second->name_.c_str());
      continue;
    }

    ROS_INFO("name: %s", iter->first.c_str());
    ROS_INFO("type: %d", iter->second->type_);
    ROS_INFO("root name: %s", iter->second->root_name_.c_str());
    ROS_INFO(" tip name: %s", iter->second->tip_name_.c_str());
    /*
    ROS_INFO("redundancy name: %s", iter->second->redundancy_name_.c_str());
    ROS_INFO("planning joints: ");
    for(size_t i = 0; i < iter->second->joint_names_.size(); ++i)
      ROS_INFO(" %d: %s", int(i), iter->second->joint_names_[i].c_str());
    */
    ROS_INFO("collision links: ");
    for(size_t i = 0; i < iter->second->links_.size(); ++i)
    {
      ROS_INFO(" name: %s", iter->second->links_[i].name_.c_str());
      ROS_INFO(" root: %s", iter->second->links_[i].root_name_.c_str());
      if(iter->second->type_ == sbpl_arm_planner::Group::SPHERES)
      {
        ROS_INFO(" spheres: %d", int(iter->second->links_[i].spheres_.size()));
        for(size_t j = 0; j < iter->second->links_[i].spheres_.size(); ++j)
          ROS_INFO("  [%s] x: %0.3f y: %0.3f z: %0.3f radius: %0.3f priority: %d", iter->second->links_[i].spheres_[j].name.c_str(), iter->second->links_[i].spheres_[j].v.x(), iter->second->links_[i].spheres_[j].v.y(), iter->second->links_[i].spheres_[j].v.z(), iter->second->links_[i].spheres_[j].radius, iter->second->links_[i].spheres_[j].priority);
      }
      else if(iter->second->type_ == sbpl_arm_planner::Group::VOXELS)
      {
        ROS_INFO(" voxels: %d", int(iter->second->links_[i].voxels_.v.size()));
        for(size_t j = 0; j < iter->second->links_[i].voxels_.v.size(); ++j)
          ROS_DEBUG("  [%d] x: %0.3f y: %0.3f z: %0.3f", int(j), iter->second->links_[i].voxels_.v[j].x(), iter->second->links_[i].voxels_.v[j].y(), iter->second->links_[i].voxels_.v[j].z());
      }
      if(i < iter->second->links_.size()-1)
        ROS_INFO(" ---");
    }
    ROS_INFO(" ");
    if(iter->second->type_ == sbpl_arm_planner::Group::SPHERES)
    {
      ROS_INFO("sorted spheres: ");
      for(size_t j = 0; j < iter->second->spheres_.size(); ++j)
      {
        ROS_INFO("  [%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", iter->second->spheres_[j]->name.c_str(), iter->second->spheres_[j]->v.x(), iter->second->spheres_[j]->v.y(), iter->second->spheres_[j]->v.z(), iter->second->spheres_[j]->radius, iter->second->spheres_[j]->priority);
      }
      ROS_INFO(" ");
    }
    ROS_INFO("kdl chain: ");
    for(size_t j = 0; j < iter->second->chains_.size(); ++j)
      leatherman::printKDLChain(iter->second->chains_[j], "chain " + boost::lexical_cast<std::string>(j));
    ROS_INFO(" ");
  }
}

void SBPLCollisionModel::printSpheres(std::string group_name)
{
  if(!group_config_map_[group_name]->init_)
  {
    ROS_ERROR("Failed to print the collision spheres because the %s group is not initialized.", group_name.c_str());
    return;
  }

  ROS_INFO("\n%s", group_name.c_str());
  for(size_t i = 0; i < group_config_map_[group_name]->spheres_.size(); ++i)
  {
    ROS_INFO("[%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", group_config_map_[group_name]->spheres_[i]->name.c_str(), group_config_map_[group_name]->spheres_[i]->v.x(), group_config_map_[group_name]->spheres_[i]->v.y(), group_config_map_[group_name]->spheres_[i]->v.z(), group_config_map_[group_name]->spheres_[i]->radius, group_config_map_[group_name]->spheres_[i]->priority);
  }
  ROS_INFO(" ");
}

void SBPLCollisionModel::sortSpheres(std::string group_name)
{
  sort(group_config_map_[group_name]->spheres_.begin(), group_config_map_[group_name]->spheres_.end(), sortSphere);
  ROS_DEBUG("Sorted the spheres by priority");
}

/*
int SBPLCollisionModel::getSegmentNumber(std::string &name, KDL::Chain &chain)
{
  for(size_t k = 0; k < chain.getNrOfSegments(); ++k)
  {
    if(chain.getSegment(k).getName().compare(name) == 0)
      return k;
  }
  ROS_DEBUG("Failed to find %s segment in the chain.", name.c_str());
  return -1;
}
*/

bool SBPLCollisionModel::getFrameInfo(std::string &name, std::string group_name, int &chain, int &segment)
{
  Group* group = group_config_map_[group_name];

  for(size_t i = 0; i < group->chains_.size(); ++i)
  {
    for(size_t k = 0; k < group->chains_[i].getNrOfSegments(); ++k)
    {
      if(group->chains_[i].getSegment(k).getName().compare(name) == 0)
      {
        chain = i;
        segment = k;
        return true;
      }
    }
  }
  ROS_DEBUG("Failed to find %s segment in any chain in %s group.", name.c_str(), group_name.c_str());
  return false;
}

bool SBPLCollisionModel::initAllGroups()
{
  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter)
  {
    if(!initKDLChainForGroup(iter->second))
      return false;


    if(!initGroup(iter->second))
      return false;
  }
  return true;
}

bool SBPLCollisionModel::initGroup(Group* group)
{
  if(group->type_ == sbpl_arm_planner::Group::VOXELS)
    return initVoxelGroup(group);

  // assign the kdl segment numbers to each sphere
  for(size_t i = 0; i < group->links_.size(); ++i)
  {
    int seg;
    if(!leatherman::getSegmentIndex(group->chains_[group->links_[i].i_chain_], group->links_[i].root_name_, seg))
      return false;

    for(size_t j = 0; j < group->links_[i].spheres_.size(); ++j)
    {
      group->links_[i].spheres_[j].kdl_segment = seg;
      group->links_[i].spheres_[j].kdl_chain = group->links_[i].i_chain_;
    }
  }

  // fill the group's list of all the spheres
  for(size_t i = 0; i < group->links_.size(); ++i)
  {
    for(size_t j = 0; j < group->links_[i].spheres_.size(); ++j)
      group->spheres_.push_back(&(group->links_[i].spheres_[j]));
  }

  // sort the spheres by priority
  sortSpheres(group->name_);

  // populate the frames vector that stores the segments in each chain 
  group->frames_.resize(group->chains_.size());
  for(size_t i = 0; i < group->spheres_.size(); ++i)
  {
    if(std::find(group->frames_[group->spheres_[i]->kdl_chain].begin(), group->frames_[group->spheres_[i]->kdl_chain].end(), group->spheres_[i]->kdl_segment) == group->frames_[group->spheres_[i]->kdl_chain].end())
      group->frames_[group->spheres_[i]->kdl_chain].push_back(group->spheres_[i]->kdl_segment);
  }

  // debug output
  ROS_DEBUG("[%s] Frames:", group->name_.c_str());
  for(size_t i = 0; i < group->frames_.size(); ++i)
    for(size_t j = 0; j < group->frames_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] segment: %d", group->name_.c_str(), int(i), group->frames_[i][j]);

  // get order of joints for each chain
  group->jntarray_names_.resize(group->chains_.size());
  for(size_t i = 0; i < group->chains_.size(); ++i)
  {
    for(size_t j = 0; j < group->chains_[i].getNrOfSegments(); ++j)
    {
      if(group->chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0)
      {
        group->jntarray_names_[i].push_back(group->chains_[i].getSegment(j).getJoint().getName());
      }
    }
  }

  // debug output
  ROS_DEBUG("[%s] Order of Joints in Joint Array input to the FK Solver:", group->name_.c_str());
  for(size_t i = 0; i < group->jntarray_names_.size(); ++i)
    for(size_t j = 0; j < group->jntarray_names_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] %d: %s", group->name_.c_str(), int(i), int(j), group->jntarray_names_[i][j].c_str());
 
  // initialize the sizes of the JntArrays
  group->joint_positions_.resize(group->chains_.size());
  for(size_t i = 0; i < group->chains_.size(); ++i)
  {
    group->joint_positions_[i].resize(group->jntarray_names_[i].size());
    KDL::SetToZero(group->joint_positions_[i]);
  }
    
  group->init_ = true;
  return true;
}

bool SBPLCollisionModel::initVoxelGroup(Group* group)
{
  // get link voxels && assign the kdl segment numbers to each link 
  for(size_t i = 0; i < group->links_.size(); ++i)
  {
    if(!getLinkVoxels(group->links_[i].root_name_, group->links_[i].voxels_.v))
    {
      ROS_ERROR("Failed to retrieve voxels for link '%s' in group '%s'", group->links_[i].root_name_.c_str(), group->name_.c_str());
      return false;
    }
    ROS_INFO("Retrieved %d voxels for link '%s'", int(group->links_[i].voxels_.v.size()), group->links_[i].root_name_.c_str());
    int seg;
    if(!leatherman::getSegmentIndex(group->chains_[group->links_[i].i_chain_], group->links_[i].root_name_, seg))
      return false;

    group->links_[i].voxels_.kdl_segment = seg;
    group->links_[i].voxels_.kdl_chain = group->links_[i].i_chain_;
  }

  
  // populate the frames vector that stores the segments in each chain 
  group->frames_.resize(group->chains_.size());
  for(size_t i = 0; i < group->links_.size(); ++i)
    group->frames_[group->links_[i].voxels_.kdl_chain].push_back(group->links_[i].voxels_.kdl_segment);

  // debug output
  ROS_WARN("[%s] Frames:", group->name_.c_str());
  for(size_t i = 0; i < group->frames_.size(); ++i)
    for(size_t j = 0; j < group->frames_[i].size(); ++j)
      ROS_WARN("[%s] [chain %d] segment: %d", group->name_.c_str(), int(i), group->frames_[i][j]);
  

  // get order of joints for each chain
  group->jntarray_names_.resize(group->chains_.size());
  for(size_t i = 0; i < group->chains_.size(); ++i)
  {
    for(size_t j = 0; j < group->chains_[i].getNrOfSegments(); ++j)
    {
      if(group->chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0)
      {
        group->jntarray_names_[i].push_back(group->chains_[i].getSegment(j).getJoint().getName());
      }
    }
  }

  // debug output
  ROS_WARN("[%s] Order of Joints in Joint Array input to the FK Solver:", group->name_.c_str());
  for(size_t i = 0; i < group->jntarray_names_.size(); ++i)
    for(size_t j = 0; j < group->jntarray_names_[i].size(); ++j)
      ROS_WARN("[%s] [chain %d] %d: %s", group->name_.c_str(), int(i), int(j), group->jntarray_names_[i][j].c_str());
 
  // initialize the sizes of the JntArrays
  group->joint_positions_.resize(group->chains_.size());
  for(size_t i = 0; i < group->chains_.size(); ++i)
  {
    group->joint_positions_[i].resize(group->jntarray_names_[i].size());
    KDL::SetToZero(group->joint_positions_[i]);
  }
    
  group->init_ = true;
  return true;
}

void SBPLCollisionModel::initFKSolvers(Group* group)
{
  group->solvers_.resize(group->chains_.size());

  for(size_t i = 0; i < group->chains_.size(); ++i)
  {
    group->solvers_[i] = new KDL::ChainFkSolverPos_recursive(group->chains_[i]);
    ROS_INFO("[%s] Instantiated a forward kinematics solver for chain #%d for the %s with %d joints.", group->name_.c_str(), int(i), group->name_.c_str(), group->chains_[i].getNrOfJoints());
  }
}

bool SBPLCollisionModel::computeDefaultGroupFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame> > &frames)
{
  return computeGroupFK(angles, dgroup_, frames);
}

bool SBPLCollisionModel::computeGroupFK(const std::vector<double> &angles, Group* group, std::vector<std::vector<KDL::Frame> > &frames)
{
  frames.resize(group->chains_.size());
  for(int i = 0; i < int(group->frames_.size()); ++i)
  {
    frames[i].resize(group->chains_[i].getNrOfSegments());
    for(size_t j = 0; j < group->frames_[i].size(); ++j)
    {
      ROS_DEBUG("chain: %d   frame_index: %d  frame: %d  size_of_frames_vector: %d", i, int(j), int(group->frames_[i][j]), int(frames[i].size()));
      if(!computeFK(angles, group, i, group->frames_[i][j]+1, frames[i][group->frames_[i][j]]))
        return false;
    }
  }
  return true;
}

bool SBPLCollisionModel::computeFK(const std::vector<double> &angles, Group* group, int chain, int segment, KDL::Frame &frame)
{
  // sort elements of input angles into proper positions in the JntArray
  //printf("group->angles_to_jntarray_[chain][i]: ");
  for(size_t i = 0; i < angles.size(); ++i)
  {
    if(group->angles_to_jntarray_[chain][i] == -1)
      continue;
    group->joint_positions_[chain](group->angles_to_jntarray_[chain][i]) = angles[i];
    //printf("%d  ", group->angles_to_jntarray_[chain][i]);
  }
  //printf("\n");

  /*
  printf("[%s] chain: %d segment: %d jntarray: ", group->name_.c_str(), chain, segment);
  for(size_t i = 0; i < group->joint_positions_[chain].rows(); ++i)
    printf(" %f", group->joint_positions_[chain](i));
  printf("\n");
  */

  if(group->solvers_[chain]->JntToCart(group->joint_positions_[chain], frame, segment) < 0)
  {
    ROS_ERROR("JntToCart returned < 0. Exiting.");
    return false;
  }
  return true;
}

void SBPLCollisionModel::setOrderOfJointPositions(const std::vector<std::string> &joint_names, std::string group_name)
{
  // easier to type...
  Group* group = group_config_map_[group_name];

  // store the desired order of the input angles for debug information
  group->order_of_input_angles_ = joint_names;

  // for each joint, find its proper index in the JntArray for each chain's solver
  group->angles_to_jntarray_.resize(group->chains_.size());
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    bool matched = false; // kinda useless

    ROS_DEBUG("[%s] [%d] %s", group_name.c_str(), int(i), joint_names[i].c_str());
    for(size_t k = 0; k < group->chains_.size(); ++k)
    {
      group->angles_to_jntarray_[k].resize(joint_names.size(),-1);
      for(size_t j = 0; j < group->jntarray_names_[k].size(); ++j)
      {
        if(joint_names[i].compare(group->jntarray_names_[k][j]) == 0)
        {
          group->angles_to_jntarray_[k][i] = j;
          matched = true;
          break;
        }
      }
    }
    if(!matched)
      ROS_ERROR("%s was not found in either chain. Why do you send it to the forward kinematics solver?", joint_names[i].c_str());
  }

  for(size_t i = 0; i < group->angles_to_jntarray_.size(); ++i)
    for(size_t j = 0; j < group->angles_to_jntarray_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] joint: %s  index: %d", group_name.c_str(), int(i), joint_names[j].c_str(), group->angles_to_jntarray_[i][j]);
}

void SBPLCollisionModel::setJointPosition(const std::string &name, double position)
{
  /*
  for(size_t i = 0; i < dgroup_->jntarray_names_.size(); ++i)
  {
    for(size_t j = 0; j < dgroup_->jntarray_names_[i].size(); ++j)
    {
      if(name.compare(dgroup_->jntarray_names_[i][j]) == 0)
      {
        dgroup_->joint_positions_[i](j) = position;
        break;
      }
    }
  }
  */

  for(std::map<std::string, Group*>::const_iterator iter = group_config_map_.begin(); iter != group_config_map_.end(); iter++)
  {
    for(size_t i = 0; i < iter->second->jntarray_names_.size(); ++i)
    {
      for(size_t j = 0; j < iter->second->jntarray_names_[i].size(); ++j)
      {
        if(name.compare(iter->second->jntarray_names_[i][j]) == 0)
        {
          iter->second->joint_positions_[i](j) = position;
          break;
        }
      }
    }
  }

}

void SBPLCollisionModel::printDebugInfo(std::string group_name)
{
  Group* group = group_config_map_[group_name];

  ROS_INFO("[chains_] %d", int(group->chains_.size()));
  ROS_INFO("[solvers_] %d", int(group->solvers_.size()));
  ROS_INFO("[joint_positions_] %d", int(group->joint_positions_.size()));
  ROS_INFO("[frames_] %d", int(group->frames_.size()));
  for(size_t i = 0; i < group->frames_.size(); ++i)
    ROS_INFO("[frames_] [%d] %d", int(i), int(group->frames_[i].size()));
  ROS_INFO("[jntarray_names_] %d", int(group->jntarray_names_.size()));
  for(size_t i = 0; i < group->jntarray_names_.size(); ++i)
    ROS_INFO("[jntarray_names_] [%d] %d", int(i), int(group->jntarray_names_[i].size()));
  ROS_INFO("[angles_to_jntarray_] %d", int(group->angles_to_jntarray_.size()));
  for(size_t i = 0; i < group->angles_to_jntarray_.size(); ++i)
    ROS_INFO("[angles_to_jntarray_] [%d] %d", int(i), int(group->angles_to_jntarray_[i].size()));
}

void SBPLCollisionModel::getDefaultGroupSpheres(std::vector<Sphere *> &spheres)
{
  spheres = dgroup_->spheres_;
}

bool SBPLCollisionModel::getJointLimits(std::string root_name, std::string tip_name, std::vector<std::string> &joint_names, std::vector<double> &min_limits, std::vector<double> &max_limits, std::vector<bool> &continuous)
{
  unsigned int num_joints = 0;

  boost::shared_ptr<const urdf::Link> link = urdf_->getLink(tip_name);
  while(link && link->name != root_name)
  {
    boost::shared_ptr<const urdf::Joint> joint = urdf_->getJoint(link->parent_joint->name);
    ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      num_joints++;
    }
    link = urdf_->getLink(link->getParent()->name);
  }
  ROS_INFO("%d joints found.", num_joints);

  min_limits.resize(num_joints);
  max_limits.resize(num_joints);
  joint_names.resize(num_joints);
  continuous.resize(num_joints, false);

  link = urdf_->getLink(tip_name);
  unsigned int i = 0;
  while(link && i < num_joints)
  {
    boost::shared_ptr<const urdf::Joint> joint = urdf_->getJoint(link->parent_joint->name);
    ROS_DEBUG( "getting bounds for joint: [%s]", joint->name.c_str() );
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      if( joint->type != urdf::Joint::CONTINUOUS )
      {
        joint_names[num_joints-i-1] = joint->name;
        continuous[num_joints-i-1] = false;

        if(joint->safety == NULL)
        {
          min_limits[num_joints-i-1] = joint->limits->lower;
          max_limits[num_joints-i-1] = joint->limits->upper;
        }
        else
        {
          min_limits[num_joints-i-1] = joint->limits->lower;
          max_limits[num_joints-i-1] = joint->limits->upper;
          // min_limits[num_joints-i-1] = joint->safety->soft_lower_limit;
          // max_limits[num_joints-i-1] = joint->safety->soft_upper_limit;
        }
      }
      else
      {
        joint_names[num_joints-i-1] = joint->name;
        min_limits[num_joints-i-1] = -M_PI;
        max_limits[num_joints-i-1] = M_PI;
        continuous[num_joints-i-i] = true;
      }
      ROS_INFO("[%s] min: %0.3f  max: %0.3f", joint_names[num_joints-i-1].c_str(), min_limits[num_joints-i-1], max_limits[num_joints-i-1]);
      i++;
    }
    link = urdf_->getLink(link->getParent()->name);
  }

  return true;
}

bool SBPLCollisionModel::getJointLimits(std::string group_name, std::string joint_name, double &min_limit, double &max_limit, bool &continuous)
{
  bool found_joint = false;
  if(group_config_map_.find(group_name) == group_config_map_.end())
    return false;
  if(!group_config_map_[group_name]->init_)
    return false;

  Group* g = group_config_map_[group_name];
  boost::shared_ptr<const urdf::Link> link = urdf_->getLink(g->tip_name_);
  while(link && (link->name != g->root_name_) && !found_joint)
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
            min_limit = joint->limits->lower;
            max_limit = joint->limits->upper;
            //min_limit = joint->safety->soft_lower_limit;
            //max_limit = joint->safety->soft_upper_limit;
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

std::string SBPLCollisionModel::getReferenceFrame(std::string group_name)
{
  if(group_config_map_.find(group_name) == group_config_map_.end())
    return "";
  if(!group_config_map_[group_name]->init_)
    return "";
  return group_config_map_[group_name]->root_name_;
}

bool SBPLCollisionModel::getLinkVoxels(std::string name, std::vector<KDL::Vector> &voxels)
{
  ROS_ERROR("link '%s'", name.c_str());
  boost::shared_ptr<const urdf::Link> link = urdf_->getLink(name);
  if(link == NULL)
  {
    ROS_ERROR("Failed to find link '%s' in URDF.", name.c_str());
    return false;
  }
  if(link->collision == NULL)
  {
    ROS_ERROR("Failed to find collision field for link '%s' in URDF.", link->name.c_str());
    return false;
  }
  if(link->collision->geometry == NULL)
  {
    ROS_ERROR("Failed to find geometry for link '%s' in URDF. (group: %s)", name.c_str(), link->collision->group_name.c_str());
    return false;
  }

  boost::shared_ptr<const urdf::Geometry> geom = link->collision->geometry;

  if(geom->type == urdf::Geometry::MESH)
  {
    std::vector<int> triangles;
    std::vector<geometry_msgs::Point> vertices;
    std::vector<std::vector<double> > v;
    urdf::Mesh* mesh = (urdf::Mesh*) geom.get();
    if(!leatherman::getMeshComponentsFromResource(mesh->filename, triangles, vertices))
    {
      ROS_ERROR("Failed to get mesh from file. (%s)", mesh->filename.c_str());
      return false;
    }
    ROS_INFO("mesh: %s  triangles: %u  vertices: %u", name.c_str(), int(triangles.size()), int(vertices.size()));
    sbpl::Voxelizer::voxelizeMesh(vertices, triangles, RESOLUTION, v, false, 100000); 
    ROS_INFO("mesh: %s  voxels: %u", name.c_str(), int(v.size()));
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else if(geom->type == urdf::Geometry::BOX)
  {
    std::vector<std::vector<double> > v;
    urdf::Box* box = (urdf::Box*) geom.get();
    sbpl::Voxelizer::voxelizeBox(box->dim.x, box->dim.y, box->dim.z, RESOLUTION, v, false); 
    ROS_INFO("box: %s  voxels: %u", name.c_str(), int(v.size()));
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else if(geom->type == urdf::Geometry::SPHERE)
  {
    std::vector<std::vector<double> > v;
    urdf::Sphere* sph = (urdf::Sphere*) geom.get();
    sbpl::Voxelizer::voxelizeSphere(sph->radius, RESOLUTION, v, false); 
    ROS_INFO("sphere: %s  voxels: %u", name.c_str(), int(v.size()));
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else
  {
    ROS_ERROR("Failed to get voxels for link '%s'.", name.c_str());
    return false;
  }

  return true;
}

Group* SBPLCollisionModel::getGroup(std::string name)
{
  Group* r = NULL;
  if(group_config_map_.find(name) == group_config_map_.end())
    return r;
  return group_config_map_[name];
}
}
