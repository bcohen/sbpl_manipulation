#ifndef _COLLISION_CHECKER_H_
#define _COLLISION_CHECKER_H_

#include <ros/console.h>
#include <angles/angles.h>
#include <string>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
//#include <planning_models_loader/kinematic_model_loader.h>
#include <planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <collision_distance_field/hybrid_collision_robot.h>
#include <collision_distance_field/hybrid_collision_world.h>
#include <collision_distance_field_ros/hybrid_collision_robot_ros.h>
#include <sbpl_geometry_utils/Interpolator.h>

class CollisionChecker
{
  public:
    
    CollisionChecker(std::string group_name);
    ~CollisionChecker();

    bool setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

    bool checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, unsigned char &dist);

    bool checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, int &path_length, int &num_checks, unsigned char &dist);

    bool interpolatePath(const std::vector<double>& start,
                         const std::vector<double>& end,
                         const std::vector<double>& inc,
                         std::vector<std::vector<double> >& path);

  private:

    bool first_run_;
    std::string group_name_;

    std::vector<double> inc_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;

    //boost::shared_ptr<planning_models_loader::KinematicModelLoader> kinematic_model_loader_;
    boost::shared_ptr<collision_detection::GroupStateRepresentation> gsr_;
    
    planning_models::KinematicState::JointStateGroup* jsg_;
    planning_models::KinematicState* kstate_;

    collision_detection::CollisionRequest req_;
    collision_detection::CollisionResult res_;

    const collision_detection::CollisionWorldHybrid* hy_world_;
    const collision_detection::CollisionRobotHybridROS* hy_robot_;
    planning_scene::PlanningSceneConstPtr planning_scene_;
};

#endif

