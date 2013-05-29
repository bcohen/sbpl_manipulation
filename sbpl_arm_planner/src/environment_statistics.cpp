#include <sbpl_arm_planner/environment_statistics.h>

using namespace sbpl_arm_planner;

EnvironmentStatistics::EnvironmentStatistics()
{
  resetAllCheckCounters();
  solver_type_names_.push_back("none");
  solver_type_names_.push_back("os");
  solver_type_names_.push_back("ik");
  solver_type_names_.push_back("ik_search");
  solver_type_names_.push_back("total");
}
 
void EnvironmentStatistics::resetSingleCheckCounters()
{
  nchecks_per_solver_per_succ_.clear();
  nchecks_per_solver_per_succ_.resize(solver_types::NUMBER_OF_SOLVERS, 0);
} 

void EnvironmentStatistics::resetSingleExpandCounter()
{
  resetSingleCheckCounters();
  nchecks_per_solver_per_single_expand_.clear();
  nchecks_per_solver_per_single_expand_.resize(solver_types::NUMBER_OF_SOLVERS+1, 0);
}

void EnvironmentStatistics::resetAllCheckCounters()
{
  resetSingleExpandCounter();
  total_checks_per_solver_.clear();
  avg_checks_per_solver_per_expand_.clear();
  nchecks_per_solver_per_expand_.clear();
} 

void EnvironmentStatistics::resetSolverCounters()
{
  total_num_solver_used_.clear();
  total_num_solver_used_.resize(solver_types::NUMBER_OF_SOLVERS, 0);
}

void EnvironmentStatistics::resetAllRecordedData()
{
  resetAllCheckCounters();
  resetSolverCounters();
  resetStatMap();
}

void EnvironmentStatistics::updateExpand()
{
  for(size_t i = 0; i < nchecks_per_solver_per_succ_.size(); ++i)
  {
    nchecks_per_solver_per_single_expand_[i] += nchecks_per_solver_per_succ_[i];
    nchecks_per_solver_per_single_expand_.back() += nchecks_per_solver_per_succ_[i];
  }
  resetSingleCheckCounters();
}

void EnvironmentStatistics::logChecksForExpand()
{
  nchecks_per_solver_per_expand_.push_back(nchecks_per_solver_per_single_expand_);
  resetSingleExpandCounter();
}   

void EnvironmentStatistics::updateAverageChecksPerExpand()
{
  if(nchecks_per_solver_per_expand_.empty())
    return;
  total_checks_per_solver_.resize(nchecks_per_solver_per_expand_[0].size(), 0);
  avg_checks_per_solver_per_expand_.resize(nchecks_per_solver_per_expand_[0].size(), 0);
  if(nchecks_per_solver_per_expand_.empty())
  {
    ROS_ERROR("[stats] Can't compute the avgs & totals because we have recorded 0 checks so far.");
    return;
  }
  for(size_t i = 0; i < total_checks_per_solver_.size(); ++i)
  {
    total_checks_per_solver_[i] = accumulateColumn(nchecks_per_solver_per_expand_, i);
    avg_checks_per_solver_per_expand_[i] = total_checks_per_solver_[i] / nchecks_per_solver_per_expand_.size();
  } 
} 

int EnvironmentStatistics::accumulateColumn(const std::vector<std::vector<int> > &v, int index)
{
  if(v.empty())
    return 0;

  if(index >= int(v[0].size()))
    return 0;
    
  int sum = 0;
  for(size_t i = 0; i < v.size(); ++i)
    sum += v[i][index];

  return sum;
}

void EnvironmentStatistics::printChecksPerSolverPerSingleExpand()
{
  std::string str;
  str = "[stats] [checks_per_solver_per_single_expand] ";
  for(size_t i = 0; i < nchecks_per_solver_per_single_expand_.size(); ++i)
    str += solver_type_names_[i] + ": " + boost::lexical_cast<std::string>(nchecks_per_solver_per_single_expand_[i]) + "  ";
  ROS_INFO("%s", str.c_str());
}

void EnvironmentStatistics::printTotalSolverUsedSummary()
{
  std::string str;
  str = "[stats] [total_#_solver_used] ";
  for(size_t i = 0; i < total_num_solver_used_.size(); ++i)
    str += solver_type_names_[i] + ": " + boost::lexical_cast<std::string>(total_num_solver_used_[i]) + "  ";
  ROS_INFO("%s", str.c_str());
}

void EnvironmentStatistics::printTotalChecksPerSolverSummary()
{
  if(avg_checks_per_solver_per_expand_.size() < solver_type_names_.size() || total_checks_per_solver_.size() < solver_type_names_.size())
  {
    ROS_ERROR("Stats are broken");
    return;
  }
  ROS_INFO("[stats] # expands: %d", int(nchecks_per_solver_per_expand_.size()));
  std::string str;
  str = "[stats] [avg_checks_per_solver_per_expand] ";
  for(size_t i = 0; i < solver_type_names_.size(); ++i)
    str += solver_type_names_[i] + ": " + boost::lexical_cast<std::string>(avg_checks_per_solver_per_expand_[i]) + "  ";
  ROS_INFO("%s", str.c_str());
  str =  "[stats] [total_checks_per_solver] ";
  for(size_t i = 0; i < solver_type_names_.size(); ++i)
    str += solver_type_names_[i] + ": " + boost::lexical_cast<std::string>(total_checks_per_solver_[i]) + "  ";
  ROS_INFO("%s", str.c_str());
}

void EnvironmentStatistics::setStat(std::string field, double value)
{
  ROS_DEBUG("[stats] %s: %2.2f", field.c_str(), value);
  stat_map_[field] = value;
}

void EnvironmentStatistics::incrementStat(std::string field)
{
  if(stat_map_.count(field) == 0)
    stat_map_[field] = 1;

  stat_map_[field]++;
}

void EnvironmentStatistics::resetStatMap()
{
  stat_map_.clear();
}

void EnvironmentStatistics::printStatsToFile(std::string prefix)
{
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));
  time.erase(time.size()-1, 1);

  FILE* file = fopen("/tmp/environment_stats_with_field_names.csv", "a");
  if(file != NULL)
  {
    fprintf(file, "%s, %s, ", prefix.c_str(), time.c_str());
    fprintf(file, "stats, ");
    for(std::map<std::string, double>::const_iterator iter = stat_map_.begin(); iter != stat_map_.end(); ++iter)
      fprintf(file, "%s, %2.3f, ", iter->first.c_str(), iter->second);
    fprintf(file, "collision_checks, ");
    for(size_t i = 0; i < avg_checks_per_solver_per_expand_.size(); ++i)
      fprintf(file, "avg-per-expand-%s, %d, ", solver_type_names_[i].c_str(), avg_checks_per_solver_per_expand_[i]);
    for(size_t i = 0; i < total_checks_per_solver_.size(); ++i)
      fprintf(file, "total-%s, %d, ", solver_type_names_[i].c_str(), total_checks_per_solver_[i]);
    fprintf(file, "successful_solver_calls, ");
    for(size_t i = 0; i < total_num_solver_used_.size(); ++i)
      fprintf(file, "total-%s, %d, ", solver_type_names_[i].c_str(), total_num_solver_used_[i]);
   fprintf(file,"\n");
    fclose(file);
  }
  else
    ROS_ERROR("[stats] Failed to open statistics file for editing.");

  file = fopen("/tmp/environment_stats.csv", "a");
  if(file != NULL)
  {
    fprintf(file, "%s, %s, ", prefix.c_str(), time.c_str());
    fprintf(file, "stats, ");
    for(std::map<std::string, double>::const_iterator iter = stat_map_.begin(); iter != stat_map_.end(); ++iter)
      fprintf(file, "%2.3f, ", iter->second);
    fprintf(file, "collision_checks, ");
    for(size_t i = 0; i < avg_checks_per_solver_per_expand_.size(); ++i)
      fprintf(file, "%d, ", avg_checks_per_solver_per_expand_[i]);
    for(size_t i = 0; i < total_checks_per_solver_.size(); ++i)
      fprintf(file, "%d, ", total_checks_per_solver_[i]);
    fprintf(file, "successful_solver_calls, ");
    for(size_t i = 0; i < total_num_solver_used_.size(); ++i)
      fprintf(file, "%d, ", total_num_solver_used_[i]);
    fprintf(file,"\n");
    fclose(file);
  }
  else
    ROS_ERROR("[stats] Failed to open statistics file for editing.");
}

