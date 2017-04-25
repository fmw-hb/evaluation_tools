#ifndef MAP_COMPARISON_NODE_H
#define MAP_COMPARISON_NODE_H

//ros includes
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
//#include <ipa_map_comparison/StartMapEval.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>

//includes for reading the backups
#include <boost/filesystem.hpp>
#include "ipa_long_term_slam/hmm_cell.h"
#include "ipa_navigation_msgs/HmmSlamBackup.h"

//includes for copying the backups
#include <sys/sendfile.h>  // sendfile
#include <fcntl.h>         // open
#include <unistd.h>        // close
#include <sys/stat.h>      // fstat
#include <sys/types.h>     // fstat


class ipa_map_comparison_node
{
public:
  ipa_map_comparison_node();
  void publish();
private:
  nav_msgs::OccupancyGrid reference_map_;
  nav_msgs::OccupancyGrid compare_map_;
  std::map<std::string, double> compareMaps();
  bool replaceBackup();
  nav_msgs::OccupancyGrid readBackupMap(std::string path);
  void writeLogFile(std::map<std::string, double> results);
  ros::Publisher pub_ref_map_;
  ros::Publisher pub_measured_map_;
  std::string eval_file_name_;
  nav_msgs::OccupancyGrid map_2d_msg_;
  nav_msgs::OccupancyGrid ref_2d_msg_;
  int number_of_neighbours_;
  float neighbourhood_score_;
  bool map_eval_started_;


};

#endif // MAP_COMPARISON_NODE_H
