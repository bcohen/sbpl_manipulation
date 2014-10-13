#include <sbpl_collision_checking/group.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <geometric_shapes/shapes.h>

#define RESOLUTION 0.02

namespace sbpl_arm_planner
{

bool sortSphere(sbpl_arm_planner::Sphere* a, sbpl_arm_planner::Sphere* b)
{
  return a->priority < b->priority;
}

Group::Group(std::string name) : name_(name)
{
  init_ = false;
}

Group::~Group()
{
  for(std::size_t i = 0; i < solvers_.size(); i++)
  {
    if(solvers_[i] != NULL)
    {
      delete solvers_[i];
      solvers_[i] = NULL;
    }
  }
}

bool Group::init(boost::shared_ptr<urdf::Model> urdf)
{
  urdf_ = urdf;
  if(!initKinematics())
    return false;

  if(type_ == sbpl_arm_planner::Group::VOXELS)
    init_ = initVoxels();
  else
    init_ = initSpheres();
  
  return init_;
}

bool Group::initKinematics()
{
  bool unincluded_links = true;
  int cnt = 0;
  std::vector<int> link_included(links_.size(),-1);
  KDL::Chain chain;
  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_, tree))
  {
    ROS_ERROR("Failed to parse tree from robot description.");
    return false;
  }

  // loop until all links are included in a single kdl chain
  while(unincluded_links && cnt < 100)
  {
    ROS_DEBUG("--------------------------------------");
    ROS_DEBUG("         %s:  %d ", name_.c_str(), cnt);
    ROS_DEBUG("--------------------------------------");
    cnt++;
    std::vector<int> num_links_per_tip(links_.size(),0);

    // compute # of links each link would include if tip of chain
    for(size_t i = 0; i < links_.size(); ++i)
    {
      // link i is already included in a chain
      if(link_included[i] > -1)
        continue;

      // link i is same as root link, set as identity
      if(root_name_.compare(links_[i].root_name_) == 0)
      {
        ROS_ERROR("The group root matches the link root. Creating an empty chain. {root: %s, tip: %s}", root_name_.c_str(), links_[i].root_name_.c_str());
        num_links_per_tip[i]++;
      }

      // create chain with link i as the tip
      if (!tree.getChain(root_name_, links_[i].root_name_, chain))
      {
        ROS_ERROR("Error: Failed to fetch the KDL chain. Exiting. (root: %s, tip: %s)", root_name_.c_str(), links_[i].root_name_.c_str());
        continue;
      }

      // count number of links included in this chain
      for(size_t j = 0; j < links_.size(); ++j)
      {
        // link j is already included in a chain
        if(link_included[j] > -1)
          continue;

        // check if link is included in this chain
        int seg;
        if(leatherman::getSegmentIndex(chain, links_[j].root_name_, seg))
          num_links_per_tip[i]++;
      }
    }

    // get chain tip that would include the most links
    int i_max = 0;
    for(size_t i = 0; i < num_links_per_tip.size(); ++i)
    {
      ROS_DEBUG("[%d]  chain_tip: %25s  included_links %d", int(i), links_[i].root_name_.c_str(), num_links_per_tip[i]);
      if(num_links_per_tip[i] > num_links_per_tip[i_max])
        i_max = i;
    }
    ROS_DEBUG("[cnt %d] Creating a chain for %s group with %s as the tip.", cnt, name_.c_str(), links_[i_max].root_name_.c_str());

    // create chain with link i_max as the tip
    if (!tree.getChain(root_name_, links_[i_max].root_name_, chain))
    {
      ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", root_name_.c_str(), links_[i_max].root_name_.c_str());
      continue;
    }

    // add chain to the group
    chains_.push_back(chain);

    // mark links that are included in this chain
    int included_links = 0;
    for(size_t i = 0; i < links_.size(); ++i)
    {
      // link i is already included in a different chain
      if(link_included[i] > -1)
      {
        included_links++;
        continue;
      }


      if(root_name_.compare(links_[i].root_name_) == 0)
      {
        ROS_ERROR("Checking which links are included in the single link chain.");
        link_included[i] = chains_.size()-1;
        links_[i].i_chain_ = chains_.size()-1;
        included_links++;
        ROS_INFO("[one_link-chain: %s] [%d] includes: %s", links_[i_max].root_name_.c_str(), included_links, links_[i].root_name_.c_str());
      }

      // check if link i is included in this chain
      int seg;
      if(leatherman::getSegmentIndex(chains_.back(), links_[i].root_name_, seg))
      {
        link_included[i] = chains_.size()-1;
        links_[i].i_chain_ = chains_.size()-1;
        included_links++;
        ROS_DEBUG("[chain: %s] [%d] includes: %s", links_[i_max].root_name_.c_str(), included_links, links_[i].root_name_.c_str());
      }
    }

    if(included_links == int(links_.size()))
      unincluded_links = false;

    ROS_DEBUG("Completed %d loops of the while loop (included_links = %d)", cnt, included_links);
  }

  for(size_t i = 0; i < link_included.size(); ++i)
    ROS_DEBUG("included link: %25s  link_root: %25s  chain: %d", links_[i].name_.c_str(), links_[i].root_name_.c_str(), link_included[i]);

  if(cnt >= 100)
    return false;

  // initialize the FK solvers
  solvers_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    solvers_[i] = new KDL::ChainFkSolverPos_recursive(chains_[i]);
    ROS_DEBUG("[%s] Instantiated a forward kinematics solver for chain #%d for the %s with %d joints.", name_.c_str(), int(i), name_.c_str(), chains_[i].getNrOfJoints());
  }

  ROS_INFO("Initialized %d chains for the %s group.", int(chains_.size()), name_.c_str());
  return true;
}


bool Group::getParams(XmlRpc::XmlRpcValue grp, XmlRpc::XmlRpcValue spheres)
{
  if(!grp.hasMember("name"))
  {
    ROS_WARN("All groups must have a name.");
    return false;
  }
  name_ = std::string(grp["name"]);

  if(!grp.hasMember("type"))
  {
    ROS_WARN("All groups must have a type. (voxels or spheres)");
    return false;
  }
  if(std::string(grp["type"]).compare("voxels") == 0)
    type_ = sbpl_arm_planner::Group::VOXELS;
  else if(std::string(grp["type"]).compare("spheres") == 0)
    type_ = sbpl_arm_planner::Group::SPHERES;
  else
  {
    ROS_ERROR("Illegal group type. (voxels or spheres)");
    return false;
  }

  if(!grp.hasMember("root_name"))
  {
    ROS_WARN("All groups must have a root_name.");
    return false;
  }
  root_name_ = std::string(grp["root_name"]);

  if(!grp.hasMember("tip_name"))
  {
    ROS_WARN("All groups must have a tip_name.");
    return false;
  }
  tip_name_ = std::string(grp["tip_name"]);

  if(grp.hasMember("collision_links"))
  {
    XmlRpc::XmlRpcValue all_links = grp["collision_links"];

    if(all_links.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("Collision links is not an array.");
      return false;
    }

    if(all_links.size() == 0) 
    {
      ROS_WARN("No links in collision links");
      return false;
    }

    Link link;
    for(int j = 0; j < all_links.size(); j++) 
    {
      link.type = type_;
      link.spheres_.clear();
      link.low_res_spheres_.clear();
      if(all_links[j].hasMember("name"))
        link.name_ = std::string(all_links[j]["name"]);
      if(all_links[j].hasMember("root"))
        link.root_name_ = std::string(all_links[j]["root"]);
      else
        ROS_WARN("No root name");

      if(type_ == sbpl_arm_planner::Group::SPHERES)
      {
        std::stringstream ss(all_links[j]["spheres"]);
        Sphere sphere;
        std::string sphere_name;
        while(ss >> sphere_name)
        {
          int i;
          for(i = 0; i < spheres.size(); ++i)
          {
            if(std::string(spheres[i]["name"]).compare(sphere_name) == 0)
            {
              sphere.name = std::string(spheres[i]["name"]);
              sphere.v.x(spheres[i]["x"]);
              sphere.v.y(spheres[i]["y"]);
              sphere.v.z(spheres[i]["z"]);
              sphere.priority = spheres[i]["priority"];
              sphere.radius = spheres[i]["radius"];
              link.spheres_.push_back(sphere);
              break;
            }
          }
          if(i == spheres.size())
          {
            ROS_ERROR("Failed to find sphere %s in the sphere list.", sphere_name.c_str());
            return false;
          }
        }
      }

      // low-res spheres
      if(type_ == sbpl_arm_planner::Group::SPHERES)
      {
        std::stringstream ss(all_links[j]["low_res_spheres"]);
        Sphere sphere;
        std::string sphere_name;
        while(ss >> sphere_name)
        {
          int i;
          for(i = 0; i < spheres.size(); ++i)
          {
            if(std::string(spheres[i]["name"]).compare(sphere_name) == 0)
            {
              sphere.name = std::string(spheres[i]["name"]);
              sphere.v.x(spheres[i]["x"]);
              sphere.v.y(spheres[i]["y"]);
              sphere.v.z(spheres[i]["z"]);
              sphere.priority = spheres[i]["priority"];
              sphere.radius = spheres[i]["radius"];
              link.low_res_spheres_.push_back(sphere);
              break;
            }
          }
          if(i == spheres.size())
          {
            ROS_ERROR("Failed to find sphere %s in the low_res_sphere list.", sphere_name.c_str());
            return false;
          }
        }
      }
      links_.push_back(link);
    }
  }
  else
  {
    ROS_ERROR("No collision_links founds. A group must have a list of 'collision_links'. Exiting.");
    return false;
  }
  return true;
}


void Group::printSpheres()
{
  if(!init_)
  {
    ROS_ERROR("Failed to print the collision spheres because the %s group is not initialized.", name_.c_str());
    return;
  }

  ROS_INFO("\n%s", name_.c_str());
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    ROS_INFO("[%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", spheres_[i]->name.c_str(), spheres_[i]->v.x(), spheres_[i]->v.y(), spheres_[i]->v.z(), spheres_[i]->radius, spheres_[i]->priority);
  }
  ROS_INFO(" ");
}

bool Group::initSpheres()
{
  if(type_ == sbpl_arm_planner::Group::VOXELS)
    return false;

  // assign the kdl segment numbers to each sphere
  for(size_t i = 0; i < links_.size(); ++i)
  {
    int seg = 0;
    if(!leatherman::getSegmentIndex(chains_[links_[i].i_chain_], links_[i].root_name_, seg))
    {
      if(root_name_.compare(links_[i].root_name_) == 0)
      {
        ROS_DEBUG("We have a one link chain. Setting seg = 0.");
        seg = -1;
      }
      else
      {
	    ROS_ERROR("Failed to get the link segment index for %s", links_[i].root_name_.c_str());
        return false;
      }
    }

    for(size_t j = 0; j < links_[i].spheres_.size(); ++j)
    {
      links_[i].spheres_[j].kdl_segment = seg + 1;
      links_[i].spheres_[j].kdl_chain = links_[i].i_chain_;
    }

    for(size_t j = 0; j < links_[i].low_res_spheres_.size(); ++j)
    {
      links_[i].low_res_spheres_[j].kdl_segment = seg + 1;
      links_[i].low_res_spheres_[j].kdl_chain = links_[i].i_chain_;
    }
  }

  // fill the group's list of all the spheres
  for(size_t i = 0; i < links_.size(); ++i)
  {
    for(size_t j = 0; j < links_[i].spheres_.size(); ++j)
      spheres_.push_back(&(links_[i].spheres_[j]));
    for(size_t j = 0; j < links_[i].low_res_spheres_.size(); ++j)
      low_res_spheres_.push_back(&(links_[i].low_res_spheres_[j]));
  }

  // sort the spheres by priority
  sort(spheres_.begin(), spheres_.end(), sortSphere);
  sort(low_res_spheres_.begin(), low_res_spheres_.end(), sortSphere);

  // populate the frames vector that stores the segments in each chain 
  frames_.resize(chains_.size());
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    if(std::find(frames_[spheres_[i]->kdl_chain].begin(), frames_[spheres_[i]->kdl_chain].end(), spheres_[i]->kdl_segment) == frames_[spheres_[i]->kdl_chain].end())
      frames_[spheres_[i]->kdl_chain].push_back(spheres_[i]->kdl_segment);
  }
  for(size_t i = 0; i < low_res_spheres_.size(); ++i)
  {
    if(std::find(frames_[low_res_spheres_[i]->kdl_chain].begin(), frames_[low_res_spheres_[i]->kdl_chain].end(), low_res_spheres_[i]->kdl_segment) == frames_[low_res_spheres_[i]->kdl_chain].end())
      frames_[low_res_spheres_[i]->kdl_chain].push_back(low_res_spheres_[i]->kdl_segment);
  }

  // debug output
  ROS_DEBUG("[%s] Frames:", name_.c_str());
  for(size_t i = 0; i < frames_.size(); ++i)
    for(size_t j = 0; j < frames_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] segment: %d", name_.c_str(), int(i), frames_[i][j]);

  // get order of joints for each chain
  jntarray_names_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    for(size_t j = 0; j < chains_[i].getNrOfSegments(); ++j)
    {
      if(chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0)
      {
        jntarray_names_[i].push_back(chains_[i].getSegment(j).getJoint().getName());
      }
    }
  }

  // debug output
  ROS_DEBUG("[%s] Order of Joints in Joint Array input to the FK Solver:", name_.c_str());
  for(size_t i = 0; i < jntarray_names_.size(); ++i)
    for(size_t j = 0; j < jntarray_names_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] %d: %s", name_.c_str(), int(i), int(j), jntarray_names_[i][j].c_str());

  // initialize the sizes of the JntArrays
  joint_positions_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    joint_positions_[i].resize(jntarray_names_[i].size());
    KDL::SetToZero(joint_positions_[i]);
  }
  return true;
}

bool Group::initVoxels()
{
  // get link voxels && assign the kdl segment numbers to each link 
  for(size_t i = 0; i < links_.size(); ++i)
  {
    if(!getLinkVoxels(links_[i].root_name_, links_[i].voxels_.v))
    {
      ROS_ERROR("Failed to retrieve voxels for link '%s' in group '%s'", links_[i].root_name_.c_str(), name_.c_str());
      return false;
    }
    ROS_DEBUG("Retrieved %d voxels for link '%s'", int(links_[i].voxels_.v.size()), links_[i].root_name_.c_str());
    int seg = 0;
    if(!leatherman::getSegmentIndex(chains_[links_[i].i_chain_], links_[i].root_name_, seg))
    {
      ROS_ERROR("When retrieving group voxels, getSegmentIndex() failed. The group root must be the same as the link root. {root: %s, tips: %s}", root_name_.c_str(),  links_[i].root_name_.c_str());
      seg = -1;
      //return false;
    }

    links_[i].voxels_.kdl_segment = seg+1;
    links_[i].voxels_.kdl_chain = links_[i].i_chain_;
  }

  // populate the frames vector that stores the segments in each chain 
  frames_.resize(chains_.size());
  for(size_t i = 0; i < links_.size(); ++i)
    frames_[links_[i].voxels_.kdl_chain].push_back(links_[i].voxels_.kdl_segment);

  // debug output
  ROS_DEBUG("[%s] Frames:", name_.c_str());
  for(size_t i = 0; i < frames_.size(); ++i)
    for(size_t j = 0; j < frames_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] segment: %d", name_.c_str(), int(i), frames_[i][j]);


  // get order of joints for each chain
  jntarray_names_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    for(size_t j = 0; j < chains_[i].getNrOfSegments(); ++j)
    {
      if(chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0)
      {
        jntarray_names_[i].push_back(chains_[i].getSegment(j).getJoint().getName());
      }
    }
  }

  // debug output
  ROS_DEBUG("[%s] Order of Joints in Joint Array input to the FK Solver:", name_.c_str());
  for(size_t i = 0; i < jntarray_names_.size(); ++i)
    for(size_t j = 0; j < jntarray_names_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] %d: %s", name_.c_str(), int(i), int(j), jntarray_names_[i][j].c_str());

  // initialize the sizes of the JntArrays
  joint_positions_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    joint_positions_[i].resize(jntarray_names_[i].size());
    KDL::SetToZero(joint_positions_[i]);
  }
  return true;
}

bool Group::computeFK(const KDL::JntArray &angles, int chain, int segment, KDL::Frame &frame)
{
  if(segment == 0)
  {
    ROS_DEBUG("segment is 0!");
    frame = KDL::Frame::Identity();
  }
  else
  {
    if(solvers_[chain]->JntToCart(angles, frame, segment) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting.");
      return false;
    }
  }

  frame = T_root_to_world_ * frame;
  return true;
}

bool Group::computeFK(const std::vector<double> &angles, int chain, int segment, KDL::Frame &frame)
{
  if(segment == 0)
  {
    ROS_DEBUG("segment is 0!");
    frame = KDL::Frame::Identity();
  }
  else
  {
    // sort elements of input angles into proper positions in the JntArray
    for(size_t i = 0; i < angles.size(); ++i)
    {
      if(angles_to_jntarray_[chain][i] == -1)
        continue;
      joint_positions_[chain](angles_to_jntarray_[chain][i]) = angles[i];
    }
//    for(size_t k = 0; k < joint_positions_[chain].rows(); ++k)
//      ROS_WARN("[%s] [%d] chain: %d  joint_position: %0.3f", name_.c_str(), int(k), chain, joint_positions_[chain](k));

    if(solvers_[chain]->JntToCart(joint_positions_[chain], frame, segment) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting.");
      return false;
    }
  }

  frame = T_root_to_world_ * frame;
  return true;
}

bool Group::computeFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame> > &frames)
{
  frames.resize(chains_.size());
  for(int i = 0; i < int(frames_.size()); ++i)
  {
    frames[i].resize(chains_[i].getNrOfSegments()+1);

    // sort elements of input angles into proper positions in the KDL::JntArray
    for(size_t k = 0; k < angles.size(); ++k)
    {
      if(angles_to_jntarray_[i][k] == -1)
        continue;
      joint_positions_[i](angles_to_jntarray_[i][k]) = angles[k];
    }

    for(size_t j = 0; j < frames_[i].size(); ++j)
    {
      if(!computeFK(joint_positions_[i], i, frames_[i][j]/*+1*/, frames[i][frames_[i][j]]))
      //if(!computeFK(angles, i, frames_[i][j]/*+1*/, frames[i][frames_[i][j]]))
        return false;
    }
  }
  return true;
}

void Group::setOrderOfJointPositions(const std::vector<std::string> &joint_names)
{
  // store the desired order of the input angles for debug information
  order_of_input_angles_ = joint_names;

  // for each joint, find its proper index in the JntArray for each chain's solver
  angles_to_jntarray_.resize(chains_.size());
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    bool matched = false; // kinda useless

    ROS_DEBUG("[%s] [%d] %s", name_.c_str(), int(i), joint_names[i].c_str());
    for(size_t k = 0; k < chains_.size(); ++k)
    {
      angles_to_jntarray_[k].resize(joint_names.size(),-1);
      for(size_t j = 0; j < jntarray_names_[k].size(); ++j)
      {
        if(joint_names[i].compare(jntarray_names_[k][j]) == 0)
        {
          angles_to_jntarray_[k][i] = j;
          matched = true;
          break;
        }
      }
    }
    if(!matched)
      ROS_ERROR("%s was not found in either chain. Why do you send it to the forward kinematics solver?", joint_names[i].c_str());
  }

  for(size_t i = 0; i < angles_to_jntarray_.size(); ++i)
    for(size_t j = 0; j < angles_to_jntarray_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] joint: %s  index: %d", name_.c_str(), int(i), joint_names[j].c_str(), angles_to_jntarray_[i][j]);
}

void Group::setJointPosition(const std::string &name, double position)
{
  for(std::size_t i = 0; i < jntarray_names_.size(); ++i)
  {
    for(std::size_t j = 0; j < jntarray_names_[i].size(); ++j)
    {
      if(name.compare(jntarray_names_[i][j]) == 0)
      {
        joint_positions_[i](j) = position;
        //ROS_WARN("[%s] chain: %d  index: %d  name: %s  position: %0.3f", name_.c_str(), int(i), int(j), name.c_str(), position);
        break;
      }
    }
  }
}

std::string Group::getReferenceFrame()
{
  if(!init_)
    return "";
  return root_name_;
}

std::string Group::getName()
{
  return name_;
}

void Group::getSpheres(std::vector<Sphere*> &spheres, bool low_res)
{
  if(low_res)
    spheres = low_res_spheres_;
  else
    spheres = spheres_;
}

std::vector<Sphere*> Group::getSpheres(bool low_res)
{
  if(low_res)
    return low_res_spheres_;
  else
    return spheres_;
}

bool Group::getLinkVoxels(std::string name, std::vector<KDL::Vector> &voxels)
{
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

  //TODO: Add way to choose collision geometry or visual geometry
    
  geometry_msgs::Pose p;
  
   
  boost::shared_ptr<const urdf::Geometry> geom = link->visual->geometry;
  p.position.x = link->visual->origin.position.x;
  p.position.y = link->visual->origin.position.y;
  p.position.z = link->visual->origin.position.z;
  link->visual->origin.rotation.getQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  
 
  /*  
  boost::shared_ptr<const urdf::Geometry> geom = link->collision->geometry;
  p.position.x = link->collision->origin.position.x;
  p.position.y = link->collision->origin.position.y;
  p.position.z = link->collision->origin.position.z;
  link->collision->origin.rotation.getQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  */
  
  //leatherman::printPoseMsg(p, name + "_POSE");
  voxels.clear();
  if(geom->type == urdf::Geometry::MESH)
  {
    geometry_msgs::Vector3 scale;
    scale.x = 1; scale.y = 1; scale.z = 1;
    std::vector<int> triangles;
    std::vector<geometry_msgs::Point> vertices;
    std::vector<std::vector<double> > v;
    urdf::Mesh* mesh = (urdf::Mesh*) geom.get();
    if(!leatherman::getMeshComponentsFromResource(mesh->filename, scale, triangles, vertices))
    {
      ROS_ERROR("Failed to get mesh from file. (%s)", mesh->filename.c_str());
      return false;
    }
    ROS_DEBUG("mesh: %s  triangles: %u  vertices: %u", name.c_str(), int(triangles.size()), int(vertices.size()));

   
    /*
	  // copied from sbpl_geometry_utils
    std::vector<geometry_msgs::Point> verticesCopy = vertices;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Translation3d transMatrix(p.position.x, p.position.y, p.position.z);
    Eigen::Affine3d transform = transMatrix * q;
    for (int i = 0; i < (int)verticesCopy.size(); i++) {
      Eigen::Vector3d vec(verticesCopy[i].x, verticesCopy[i].y, verticesCopy[i].z);
      Eigen::Vector3d transVec = transform * vec;
      verticesCopy[i].x = transVec(0);
      verticesCopy[i].y = transVec(1);
      verticesCopy[i].z = transVec(2);
    }
    sbpl::Voxelizer::voxelizeMesh(verticesCopy, triangles, RESOLUTION, v, true);
    */

    
    ROS_ERROR("To speed up computation, we are not flood filling the voxelized body meshes.");
    sbpl::Voxelizer::voxelizeMesh(vertices, triangles, p, RESOLUTION, v, false);
    

    ROS_DEBUG("mesh: %s  voxels: %u", name.c_str(), int(v.size()));
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
    sbpl::Voxelizer::voxelizeBox(box->dim.x, box->dim.y, box->dim.z, p, RESOLUTION, v, false); 
    ROS_INFO("box: %s  voxels: %u   {dims: %0.2f %0.2f %0.2f}", name.c_str(), int(v.size()), box->dim.x, box->dim.y, box->dim.z);
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else if(geom->type == urdf::Geometry::CYLINDER)
  {
    std::vector<std::vector<double> > v;
    urdf::Cylinder* cyl = (urdf::Cylinder*) geom.get();
    sbpl::Voxelizer::voxelizeCylinder(cyl->radius, cyl->length, p, RESOLUTION, v, true); 
    ROS_INFO("cylinder: %s  voxels: %u", name.c_str(), int(v.size()));
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
    sbpl::Voxelizer::voxelizeSphere(sph->radius, p, RESOLUTION, v, true); 
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

  if(voxels.empty())
  {
    ROS_ERROR("Problem voxeling '%s' link. It resulted in 0 voxels.", name.c_str()); 
  }

  return true;
}

bool Group::getFrameInfo(std::string &name, int &chain, int &segment)
{
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    for(size_t k = 0; k < chains_[i].getNrOfSegments(); ++k)
    {
      if(chains_[i].getSegment(k).getName().compare(name) == 0)
      {
        chain = i;
        segment = k;
        return true;
      }
    }
  }
  return false;
}

void Group::print()
{
  if(!init_)
  {
    ROS_ERROR("Failed to print %s group information because has not yet been initialized.", name_.c_str());
    return;
  }

  ROS_INFO("name: %s", name_.c_str());
  ROS_INFO("type: %d", type_);
  ROS_INFO("root name: %s", root_name_.c_str());
  ROS_INFO(" tip name: %s", tip_name_.c_str());
  ROS_INFO("collision links: ");
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    ROS_INFO("  name: %s", links_[i].name_.c_str());
    ROS_INFO("  root: %s", links_[i].root_name_.c_str());
    ROS_INFO(" chain: %d", links_[i].i_chain_);
    if(type_ == sbpl_arm_planner::Group::SPHERES)
    {
      ROS_INFO("spheres: %d", int(links_[i].spheres_.size()));
      for(std::size_t j = 0; j < links_[i].spheres_.size(); ++j)
        links_[i].spheres_[j].print();

      ROS_INFO("low_res_spheres: %d", int(links_[i].low_res_spheres_.size()));
      for(std::size_t j = 0; j < links_[i].low_res_spheres_.size(); ++j)
        links_[i].low_res_spheres_[j].print();
    }
    else if(type_ == sbpl_arm_planner::Group::VOXELS)
    {
      ROS_INFO(" voxels: %d chain: %d segment: %d", int(links_[i].voxels_.v.size()), links_[i].voxels_.kdl_chain, links_[i].voxels_.kdl_segment);
      for(std::size_t j = 0; j < links_[i].voxels_.v.size(); ++j)
        ROS_DEBUG("  [%d] x: %0.3f y: %0.3f z: %0.3f", int(j), links_[i].voxels_.v[j].x(), links_[i].voxels_.v[j].y(), links_[i].voxels_.v[j].z());
    }
    if(i < links_.size()-1)
      ROS_INFO(" ---");
  }
  ROS_INFO(" ");
  if(type_ == sbpl_arm_planner::Group::SPHERES)
  {
    ROS_INFO("sorted spheres: ");
    for(std::size_t j = 0; j < spheres_.size(); ++j)
      spheres_[j]->print();
    ROS_INFO("sorted low_res_spheres: ");
    for(std::size_t j = 0; j < low_res_spheres_.size(); ++j)
      low_res_spheres_[j]->print();
    ROS_INFO(" ");
  }
  ROS_INFO("kinematic chain(s): ");
  for(std::size_t j = 0; j < chains_.size(); ++j)
    leatherman::printKDLChain(chains_[j], "chain " + boost::lexical_cast<std::string>(j));
  ROS_INFO(" ");
}

void Group::printDebugInfo()
{
  ROS_INFO("[name] %s", name_.c_str());
  ROS_INFO("[chains] %d", int(chains_.size()));
  ROS_INFO("[solvers] %d", int(solvers_.size()));
  ROS_INFO("[joint_positions] %d", int(joint_positions_.size()));
  ROS_INFO("[frames] %d", int(frames_.size()));
  for(size_t i = 0; i < frames_.size(); ++i)
    ROS_INFO("[frames] [%d] %d", int(i), int(frames_[i].size()));
  ROS_INFO("[jntarray_names] %d", int(jntarray_names_.size()));
  for(size_t i = 0; i < jntarray_names_.size(); ++i)
    ROS_INFO("[jntarray_names] [%d] %d", int(i), int(jntarray_names_[i].size()));
  ROS_INFO("[angles_to_jntarray] %d", int(angles_to_jntarray_.size()));
  for(size_t i = 0; i < angles_to_jntarray_.size(); ++i)
    ROS_INFO("[angles_to_jntarray] [%d] %d", int(i), int(angles_to_jntarray_[i].size()));
}

void Group::setName(std::string name)
{
  name_ = name;
}

bool Group::setSpheres(std::vector<Sphere*> &spheres, bool low_res)
{
  if(low_res)
    low_res_spheres_ = spheres;
  else
    spheres_ = spheres;

  return true; // change to void?, forgot what I intended for here
}


}
