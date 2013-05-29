/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_collision_checking/test_sbpl_collision_space.h>

clock_t starttime;

using namespace std;

/** Initializers -------------------------------------------------------------*/
TestSBPLCollisionSpace::TestSBPLCollisionSpace() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),grid_(NULL),aviz_(NULL)
{
  angles_.resize(7,0);

  ljoint_names_.resize(7);
  rjoint_names_.resize(7);
}

TestSBPLCollisionSpace::~TestSBPLCollisionSpace()
{
  if(aviz_ != NULL)
    delete aviz_;
  if(grid_ != NULL)
    delete grid_;

  if(collision_map_filter_ != NULL)
    delete collision_map_filter_;
}

bool TestSBPLCollisionSpace::init()
{
  //planner
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("base_footprint"));
  node_handle_.param<std::string>("left_fk_service_name", left_fk_service_name_, "pr2_left_arm_kinematics/get_fk");
  node_handle_.param<std::string>("left_ik_service_name", left_ik_service_name_, "pr2_left_arm_kinematics/get_ik");
  node_handle_.param<std::string>("right_fk_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("right_ik_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_ik");
  node_handle_.param<std::string>("arm_name", arm_name_, "right_arm");

  node_handle_.param("collision_space/resolution",resolution_,0.02);
  node_handle_.param("collision_space/occupancy_grid/origin_x",originX_,-0.6);
  node_handle_.param("collision_space/occupancy_grid/origin_y",originY_,-1.15);
  node_handle_.param("collision_space/occupancy_grid/origin_z",originZ_,-0.05);
  node_handle_.param("collision_space/occupancy_grid/size_x",sizeX_,1.6);
  node_handle_.param("collision_space/occupancy_grid/size_y",sizeY_,1.8);
  node_handle_.param("collision_space/occupancy_grid/size_z",sizeZ_,1.4);

  //pr2 specific
  ljoint_names_[0] = "l_shoulder_pan_joint";
  ljoint_names_[1] = "l_shoulder_lift_joint";
  ljoint_names_[2] = "l_upper_arm_roll_joint";
  ljoint_names_[3] = "l_elbow_flex_joint";
  ljoint_names_[4] = "l_forearm_roll_joint";
  ljoint_names_[5] = "l_wrist_flex_joint";
  ljoint_names_[6] = "l_wrist_roll_joint";

  rjoint_names_[0] = "r_shoulder_pan_joint";
  rjoint_names_[1] = "r_shoulder_lift_joint";
  rjoint_names_[2] = "r_upper_arm_roll_joint";
  rjoint_names_[3] = "r_elbow_flex_joint";
  rjoint_names_[4] = "r_forearm_roll_joint";
  rjoint_names_[5] = "r_wrist_flex_joint";
  rjoint_names_[6] = "r_wrist_roll_joint";
  
  //collision space
  node_handle_.param<std::string>("collision_space/collision_map_topic", collision_map_topic_, "collision_map_occ");

  map_frame_ = "/base_footprint";

  resolution_ = 0.01;

  ROS_INFO("[test] Creating the grid.");
  grid_ = new sbpl_arm_planner::OccupancyGrid(sizeX_, sizeY_, sizeZ_, resolution_, originX_, originY_, originZ_);
  
  ROS_INFO("[test] Creating the collision space."); 
  cspace_ = new sbpl_arm_planner::SBPLCollisionSpace(grid_);

  if(arm_name_.compare("right_arm") == 0)
    cspace_->setPlanningJoints(rjoint_names_);
  else
    cspace_->setPlanningJoints(ljoint_names_);

  ROS_INFO("[test] Initializing the collision space for the %s.", arm_name_.c_str());
  cspace_->init(arm_name_);

  aviz_ = new sbpl_arm_planner::VisualizeArm(arm_name_);
  aviz_->setReferenceFrame(reference_frame_);

  collision_map_filter_ = new tf::MessageFilter<moveit_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&TestSBPLCollisionSpace::collisionMapCallback, this, _1));

  joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &TestSBPLCollisionSpace::jointStatesCallback,this);

  ROS_INFO("[test] Initialization complete.");
  return true;
}

int TestSBPLCollisionSpace::run()
{
  ros::spin();
  return 0;
}

void TestSBPLCollisionSpace::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  bool in_collision = false;
  double dist_m;
  unsigned char dist = 100;

  ROS_DEBUG("[test] joint_states callback");

  //aviz_->deleteVisualizations("right_arm_model_0", 70);
  aviz_->deleteVisualizations("collision", 20);

  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 1.6;

  // torso_lift_link
  cspace_->setJointPosition(state->name[12], state->position[12]);

  if(arm_name_.compare("right_arm") == 0)
  {
    angles_[0] = state->position[17];
    angles_[1] = state->position[18];
    angles_[2] = state->position[16];
    angles_[3] = state->position[20];
    angles_[4] = state->position[19];
    angles_[5] = state->position[21];
    angles_[6] = state->position[22];

    // r_gripper_l_finger_link
    cspace_->setJointPosition(state->name[24], state->position[24]);

    // r_gripper_r_finger_link
    cspace_->setJointPosition(state->name[25], state->position[25]);
  }
  else
  {
    angles_[0] = state->position[29];
    angles_[1] = state->position[30];
    angles_[2] = state->position[28];
    angles_[3] = state->position[32];
    angles_[4] = state->position[31];
    angles_[5] = state->position[33];
    angles_[6] = state->position[34];

    // l_gripper_l_finger_link
    cspace_->setJointPosition(state->name[36], state->position[36]);

    // l_gripper_r_finger_link
    cspace_->setJointPosition(state->name[37], state->position[37]);
  }

  if(!cspace_->checkCollision(angles_, true, false, dist))
  {
    dist_m = double(int(dist)*resolution_);
    ROS_INFO("dist = %0.3fm  COLLISION (%d spheres)", dist_m, int(cspace_->collision_spheres_.size()));
    in_collision = true;
  }
  else
  {
    dist_m = double(int(dist)*resolution_);
    ROS_INFO("dist = %0.3fm", dist_m);
  }

  std::vector<std::vector<double> > path(1,std::vector<double> (7,0)), spheres;
  path[0] = angles_;
  cspace_->getCollisionCylinders(angles_, spheres);
  aviz_->visualizeSpheres(spheres, 140, 0.8, arm_name_ + "_spheres");

  if(in_collision)
  {
    std::vector<std::vector<double> > csphere(cspace_->collision_spheres_.size(),std::vector<double> (4,0));
    for(size_t i = 0; i < cspace_->collision_spheres_.size(); ++i)
    {
      csphere[i][0] = cspace_->collision_spheres_[i].v.x();
      csphere[i][1] = cspace_->collision_spheres_[i].v.y();
      csphere[i][2] = cspace_->collision_spheres_[i].v.z();
      csphere[i][3] = cspace_->collision_spheres_[i].radius + 0.004;
    }
    aviz_->visualizeSpheres(csphere, 260, 1.0, "collision");
  }
  ROS_DEBUG("[test] joint_states callback done");
}

void TestSBPLCollisionSpace::collisionMapCallback(const moveit_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void TestSBPLCollisionSpace::updateMapFromCollisionMap(const moveit_msgs::CollisionMapConstPtr &collision_map)
{
  grid_->updateFromCollisionMap(*collision_map);
  cspace_->putCollisionObjectsInGrid();
}

void TestSBPLCollisionSpace::collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &collision_object)
{
  // for some reason, it wasn't getting all of the 'all' messages...
  if(collision_object->id.compare("all") == 0)
    cspace_->removeAllCollisionObjects();

  if(object_mutex_.try_lock())
  {
    // debug: have we seen this collision object before?
    if(object_map_.find(collision_object->id) != object_map_.end())
      ROS_DEBUG("[collisionObjectCallback] We have seen this object ('%s')  before.", collision_object->id.c_str());
    else
      ROS_DEBUG("[collisionObjectCallback] We have NOT seen this object ('%s') before.", collision_object->id.c_str());
    object_map_[collision_object->id] = (*collision_object);
    object_mutex_.unlock();
  }

  ROS_INFO("[collisionObjectCallback] %s", collision_object->id.c_str());
  cspace_->processCollisionObjectMsg((*collision_object));

  visualizeCollisionObjects();
}

void TestSBPLCollisionSpace::visualizeCollisionObjects()
{
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
  std::vector<double> color(4,1);
  color[2] = 0;

  cspace_->getCollisionObjectVoxelPoses(poses);

  points.resize(poses.size());
  for(size_t i = 0; i < poses.size(); ++i)
  {
    points[i].resize(3);
    points[i][0] = poses[i].position.x;
    points[i][1] = poses[i].position.y;
    points[i][2] = poses[i].position.z;
  }

  ROS_DEBUG("[visualizeCollisionObjects] Displaying %d known collision object voxels.", int(points.size()));
  aviz_->visualizeBasicStates(points, color, "known_objects", 0.01);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sbpl_collision_space");
  TestSBPLCollisionSpace test;

  if(!test.init())
  {
    ROS_INFO("Something is fucked");
    return 0;
  }

  return test.run();
}

