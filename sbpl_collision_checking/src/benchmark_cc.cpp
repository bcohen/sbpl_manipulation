#include <iostream>
#include <sbpl_collision_checking/test_sbpl_collision_space.h>

using namespace sbpl_arm_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark_sbpl_collision_checking");
  ros::NodeHandle nh_, ph_("~");
  int num_checks = 100;
  std::string group_name = "right_arm";

  if(argc > 1)
    num_checks = atoi(argv[1]);

  // set up the grid
  OccupancyGrid* grid_ = new OccupancyGrid(1.7, 1.9, 2.0, 0.01, -0.6, -1.25, -0.05);

  // create the collision space
  SBPLCollisionSpace* cspace_ = new SBPLCollisionSpace(grid_);
  //cspace_->setPlanningJoints(prms_.planning_joints_);
  cspace_->init(group_name);

  // get map
  moveit_msgs::CollisionMapConstPtr map = ros::topic::waitForMessage<moveit_msgs::CollisionMap>("collision_map_occ");
  grid_->updateFromCollisionMap(*map); 

  std::vector<double> angles(7,0);
  unsigned char dist = 100;

  ROS_INFO("Starting %d collision checks.", num_checks);
  clock_t start = clock();
  for(int i = 0; i < num_checks; ++i)
  {
    if(!cspace_->checkCollision(angles, false, false, dist))
    {
      ROS_INFO("In Collision");
      break;
    }
  }
  double total_time = (clock()-start)/double(CLOCKS_PER_SEC);
  ROS_INFO("       total time: %0.8fsec", total_time);
  ROS_INFO("   time per check: %0.8fsec", total_time/double(num_checks));
  ROS_INFO("checks per second: %0.8fsec", 1.0 / (total_time/double(num_checks)));

  ROS_INFO("[sanity check] # calls: %d   # calls not in collision: %d", cspace_->num_collision_checks_, cspace_->num_false_collision_checks_);

  delete cspace_; 
  delete grid_;
  return 0;
}

