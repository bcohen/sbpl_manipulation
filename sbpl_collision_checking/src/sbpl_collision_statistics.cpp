#include <sbpl_collision_checking/sbpl_collision_statistics.h>

SBPLCollisionStatistics::SBPLCollisionStatistics(sbpl_arm_planner::Group *group)
{
  group_ = group;
}

void SBPLCollisionStatistics::logSphereCollision(sbpl_arm_planner::Sphere *s, int &x, int &y, int &z, unsigned char &dist_temp)
{
  // increment number of collisions for that collision sphere
  if(col_sph_map_.find(s) == col_sph_map_.end())
    col_sph_map_[s] = 0;
  else
    col_sph_map_[s]++;
  
  /* 
  // increment number of collisions for that grid cell
  KDL::Vector v(x, y, z);
  //v.x = x; v.y = y; v.z = z;
  if(col_cell_map_.find(v) == col_cell_map_.end())
    col_cell_map_[v] = 0;
  else
    col_cell_map_[v]++;
  */
}

void SBPLCollisionStatistics::resetSphereCollisionLogs()
{ 
  col_sph_map_.clear();
  col_cell_map_.clear();
}

void SBPLCollisionStatistics::printSphereCollisionStats(std::string text)
{
  ROS_INFO("[cstats] [%s] Number of Collisions per Collision Sphere:", text.c_str());
  for(std::map<sbpl_arm_planner::Sphere*, int>::const_iterator iter = col_sph_map_.begin(); iter != col_sph_map_.end(); iter++)
  {
    ROS_INFO("[cstats] [%s] name: %5s  radius: %0.3f  collisions: %6d", text.c_str(), iter->first->name.c_str(), iter->first->radius, iter->second); 
  }

  // count number of collisions per link
  bool found = false;
  std::vector<int> num_col_per_link(group_->links_.size(), 0);
  for(std::map<sbpl_arm_planner::Sphere*, int>::const_iterator iter = col_sph_map_.begin(); iter != col_sph_map_.end(); iter++)
  {
    for(unsigned int i = 0; i < group_->links_.size(); ++i)
    {
      found = false;
      for(unsigned int j = 0; j < group_->links_[i].spheres_.size(); ++j)
      {
        if(group_->links_[i].spheres_[j] == *(iter->first))
          found = true;
      }
      if(found)
      {
        ROS_DEBUG("'%s' is in link %d, '%s'", iter->first->name.c_str(), i, group_->links_[i].name_.c_str());
        num_col_per_link[i] += iter->second;
        break;
      }
    }
    if(!found)
      ROS_ERROR("Couldn't figure out which link that %s belongs to.", iter->first->name.c_str());
  }

  int num_col = 0;
  for(unsigned int i = 0; i < num_col_per_link.size(); ++i)
    num_col += num_col_per_link[i];
  if(num_col == 0)
  {
    ROS_ERROR("[cstats] No collisions found....Something is funky?");
    return;
  }

  // print out the collisions per link
  for(unsigned int i = 0; i < group_->links_.size(); ++i)
    ROS_INFO("[cstats]  link: %18s  collisions: %6d (%2.1f%%)", group_->links_[i].name_.c_str(), num_col_per_link[i], double(num_col_per_link[i])/double(num_col) * 100.0);

  // print out the collisions for the entire gripper
  int all_gripper = 0;
  for(unsigned int i = 3; i < num_col_per_link.size(); ++i)
    all_gripper += num_col_per_link[i];
  ROS_INFO("[cstats]  link: %18s  collisions: %6d (% 2.1f%%)", "all_gripper_links", all_gripper, double(all_gripper)/double(num_col)*100.0);

  // print out the total number of collisions
  ROS_INFO("[cstats]  link: %18s  collisions: %6d", "all_collisions", num_col);
}

