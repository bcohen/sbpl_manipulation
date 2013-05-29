/*
 * Written by Gokul Subramanian, MSE in Robotics & CIS 2011
 * (Maintained by Benjamin Cohen)
 * Advised by Professor Maxim Likhachev
 * University of Pennsylvania
 * 4/14/2010
*/

#ifndef _ORIENTATION_SOLVER_
#define _ORIENTATION_SOLVER_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <sbpl_arm_planner/sbpl_arm_model.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>

using namespace std;


namespace sbpl_arm_planner
{

class RPYSolver
{
  public:

    RPYSolver(SBPLArmModel *arm, SBPLCollisionSpace *cspace);
    
    ~RPYSolver(){};

    /** \brief Function calculates the forearm roll, wrist pitch and wrist roll for 
    * the PR2 robot hand which is required to attain a certain yaw, pitch
    * and roll configuration in the world frame, starting at a certain
    * yaw, pitch and roll configuration. Do not use for other robots unless
    * the joints are of the same nature and the reference frame conventions
    * are the same.
    */
    void orientationSolver(double*, double, double, double, double, double, double, double, double, double, int);

    /** \brief check if a certain end effector orientation is feasible */
    bool isOrientationFeasible(const double* rpy, std::vector<double> &start, std::vector<double> &prefinal, std::vector<double> &final, int &num_checks);

    bool isOrientationFeasible(const double* rpy, std::vector<double> &start, std::vector<std::vector<double> > &path);

    /** \brief print stats to stdout */
    void printStats();

    /** \brief get a list of stats (number of calls etc.) */
    void getStats(std::vector<double> &stats);

  private:

    SBPLArmModel *arm_;

    SBPLCollisionSpace *cspace_;

    bool verbose_;

    bool try_both_directions_;

    int num_calls_;

    int num_invalid_predictions_;

    int num_invalid_solution_;

    int num_invalid_path_to_solution_;
};

}

#endif

