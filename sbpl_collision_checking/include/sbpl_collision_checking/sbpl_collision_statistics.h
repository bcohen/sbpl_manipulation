#ifndef _SBPL_COLLISION_STATISTICS_
#define _SBPL_COLLISION_STATISTICS_

#include <vector>
#include <sbpl_collision_checking/sbpl_collision_model.h>


class SBPLCollisionStatistics
{
  public:

    SBPLCollisionStatistics(sbpl_arm_planner::Group *group);

    ~SBPLCollisionStatistics(){};

    void logSphereCollision(sbpl_arm_planner::Sphere *s, int &x, int &y, int &z, unsigned char &dist_temp);

    void resetSphereCollisionLogs();

    void printSphereCollisionStats(std::string text);

  private:

    sbpl_arm_planner::Group *group_;

    /** \brief log the number of collisions per collision sphere **/
    std::map<sbpl_arm_planner::Sphere*, int> col_sph_map_;
            
    /** \brief log the number of collisions per cell **/
    std::map<KDL::Vector, int> col_cell_map_;

};

#endif
