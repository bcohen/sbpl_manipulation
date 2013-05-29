#ifndef _ENVIRONMENT_STATISTICS_
#define _ENVIRONMENT_STATISTICS_
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>

#define NUM_STATS 4
#define NUM_MP_TYPES 4  // regular, os, ik, ik_search

namespace solver_types {
enum Solvers
{
  NONE,
  ORIENTATION_SOLVER,
  IK,
  IK_SEARCH,
  NUMBER_OF_SOLVERS
};
}

namespace sbpl_arm_planner {

class EnvironmentStatistics 
{
  public:

    EnvironmentStatistics();
    ~EnvironmentStatistics(){};

    void resetSingleCheckCounters();
    void resetSingleExpandCounter();
    void resetAllCheckCounters();
    void resetAllRecordedData();

    /* \brief save nchecks_per_single_expand_, reset all single expand counters */
    void logChecksForExpand();
    void updateAverageChecksPerExpand();
    void updateExpand();
    
    /* kinematic solvers */
    void resetSolverCounters();
    void printTotalSolverUsedSummary();
    void printChecksPerSolverPerSingleExpand();
    void printTotalChecksPerSolverSummary();
    std::vector<int> total_num_solver_used_; 
    std::vector<int> nchecks_per_solver_per_succ_;
    void printStatsToFile(std::string prefix);

    /* \brief set any generic statistic you would like to store */
    void setStat(std::string field, double value);
    void incrementStat(std::string field);

    /* \brief reset the stat map that stores generic stats */
    void resetStatMap();

  private:

    std::map< std::string, double> stat_map_;
    int accumulateColumn(const std::vector<std::vector<int> > &v, int index);

    /* collision checks per solver type */
    std::vector<std::string> solver_type_names_; 
    std::vector<int> nchecks_per_solver_per_single_expand_;
    std::vector<std::vector<int> > nchecks_per_solver_per_expand_;
    std::vector<int> total_checks_per_solver_;
    std::vector<int> avg_checks_per_solver_per_expand_;
};

}

#endif
