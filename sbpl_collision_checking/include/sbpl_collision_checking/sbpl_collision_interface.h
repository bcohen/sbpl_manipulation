#ifndef _SBPL_COLLISION_INTERFACE_H_
#define _SBPL_COLLISION_INTERFACE_H_

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <kdl/frames.hpp>
#include <angles/angles.h>
//#include <planning_scene/planning_scene.h>
//#include <planning_models/transforms.h>
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_collision_checking/occupancy_grid.h>

namespace sbpl_arm_planner{

class SBPLCollisionInterface
{
  public:

    SBPLCollisionInterface(sbpl_arm_planner::OccupancyGrid *grid, sbpl_arm_planner::SBPLCollisionSpace *cspace);

    ~SBPLCollisionInterface();

    bool init();

//    void processPlanningScene(const planning_scene::PlanningSceneConstPtr &pscene);

    void processCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map);

    void processCollisionObjects(const std::vector<arm_navigation_msgs::CollisionObject> &collision_objects);

    void processAttachedObjects(const std::vector<arm_navigation_msgs::AttachedCollisionObject> &attached_object);

    /* Visualizations */
    void getVisualization(std::string name, std::vector<visualization_msgs::Marker> &markers);

    void getCollisionModelVisualization(const std::vector<std::vector<double> > &traj, std::string ns, std::vector<visualization_msgs::Marker> &markers);

    void getCollisionModelVisualization(const std::vector<trajectory_msgs::JointTrajectoryPoint> &traj, std::string ns, std::vector<visualization_msgs::Marker> &markers);

    void getCollisionModelVisualization(const std::vector<double> &angles, std::string ns, int id, std_msgs::ColorRGBA color, std::vector<visualization_msgs::Marker> &markers);

    void getInterpolatedPathVisualization(const std::vector<double> &start, const std::vector<double> &end, std::string ns, bool type, std::vector<visualization_msgs::Marker> &markers);

  private:

    arm_navigation_msgs::CollisionMap cmap_;
    sbpl_arm_planner::SBPLCollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    
    bool attached_object_;
    std::string reference_frame_;
    std::string attached_object_frame_;
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;
    //planning_scene::PlanningSceneConstPtr pscene_;
    //planning_models::TransformsConstPtr tf_;

    void attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name);
};

}

#endif

