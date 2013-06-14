#ifndef _COLLISION_CHECKER_
#define _COLLISION_CHECKER_

#include <ros/console.h>
#include <angles/angles.h>
#include <string>
#include <sbpl_geometry_utils/Interpolator.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/RobotState.h>
#include <visualization_msgs/MarkerArray.h>

namespace sbpl_arm_planner {

class CollisionChecker
{
  public:
    
    CollisionChecker();

    ~CollisionChecker(){};

    /* Initialization */
    virtual bool init(std::string group_name);

    virtual void setPlanningJoints(const std::vector<std::string> &planning_joints);

    /* World Update */
    virtual void setRobotState(const arm_navigation_msgs::RobotState &state);

    virtual void setPlanningScene(const arm_navigation_msgs::PlanningScene &scene);

    /* Collision Checking */
    virtual bool isStateValid(const std::vector<double> &angles, bool verbose, bool visualize, double &dist);
   
    virtual bool isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int path_length, int num_checks, double &dist);

    /* Utils */
    virtual bool interpolatePath(const std::vector<double> &start, const std::vector<double> &end, const std::vector<double> &inc, std::vector<std::vector<double> >& path);

    /* Visualizations */
    virtual visualization_msgs::MarkerArray getCollisionModelVisualization(const std::vector<double> &angles);
    
    virtual visualization_msgs::MarkerArray getVisualization(std::string type);
  
  protected:

    std::string group_name_;
    std::vector<std::string> planning_joints_;
    arm_navigation_msgs::PlanningScene planning_scene_;
    arm_navigation_msgs::RobotState robot_state_;
};

}

#endif

