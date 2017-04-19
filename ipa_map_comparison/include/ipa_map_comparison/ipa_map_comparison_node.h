#ifndef MAP_COMPARISON_NODE_H
#define MAP_COMPARISON_NODE_H

//ros includes
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <ipa_map_comparison/StartMapEval.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>


class ipa_map_comparison_node
{
public:
  ipa_map_comparison_node();
  void publish();
private:
  nav_msgs::OccupancyGrid ground_truth_map_;
  nav_msgs::OccupancyGrid map_;
  void compareMaps();
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
