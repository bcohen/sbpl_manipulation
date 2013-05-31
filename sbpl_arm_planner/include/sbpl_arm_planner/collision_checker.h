#ifndef _COLLISION_CHECKER_
#define _COLLISION_CHECKER_

#include <ros/console.h>
#include <angles/angles.h>
#include <string>
#include <geometric_shapes/shape_operations.h>
#include <sbpl_geometry_utils/Interpolator.h>

class CollisionChecker
{
  public:
    
    CollisionChecker();

    ~CollisionChecker(){};

    /* Initialization */
    virtual bool init(std::string group_name);

    void setPlanningJoints(const std::vector<std::string> &planning_joints);

    /* Collision Checking */
    virtual bool checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, double &dist);

    virtual bool checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, int &path_length, int &num_checks, double &dist);

    /* Utils */
    virtual bool interpolatePath(const std::vector<double> &start, const std::vector<double> &end, const std::vector<double> &inc, std::vector<std::vector<double> >& path);

  protected:

    std::string group_name_;
    std::vector<std::string> planning_joints;
};

#endif

