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

using namespace std;

namespace sbpl_arm_planner
{

class RPYSolver
{
  public:

    RPYSolver(double wrist_pitch_min_limit, double wrist_pitch_max_limit);

    ~RPYSolver(){};

    bool computeRPYOnly(const std::vector<double> &rpy, const std::vector<double> &start, const std::vector<double> &forearm_roll_link_pose, const std::vector<double> &endeff_link_pose, int solution_num, std::vector<double> &solution);

  private:

    double wrist_pitch_min_limit_;
    double wrist_pitch_max_limit_;

    /** \brief Function calculates the forearm roll, wrist pitch and wrist roll for 
    * the PR2 robot hand which is required to attain a certain yaw, pitch
    * and roll configuration in the world frame, starting at a certain
    * yaw, pitch and roll configuration. Do not use for other robots unless
    * the joints are of the same nature and the reference frame conventions
    * are the same.
    */
    void orientationSolver(double*, double, double, double, double, double, double, double, double, double, int);
};

}

#endif

