#include "ipa_map_comparison/ipa_map_comparison_node.h"

ipa_map_comparison_node::ipa_map_comparison_node()
{
  map_eval_started_ = false;
  ros::NodeHandle nh("~");
  ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
  neighbourhood_score_ = 0.0;
  number_of_neighbours_ = 0;
  eval_file_name_ = "test.txt";
  nh.getParam("neighbourhood_score",neighbourhood_score_);
  nh.getParam("number_of_neighbours",number_of_neighbours_);
  nh.getParam("eval_file_name", eval_file_name_);

  if (!(number_of_neighbours_ == 0 || number_of_neighbours_ == 4 || number_of_neighbours_ == 8))
  {
    ROS_WARN_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Number of Neighbours is not 0 4 or 8. Setting to 0");
    number_of_neighbours_ = 0;
  }
  if (neighbourhood_score_ > 1.0 || neighbourhood_score_ < 0.0)
  {
    ROS_WARN_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Neighbourhood Score is not between 0 and 1."
                                                      " Setting to 0");
    neighbourhood_score_ = 0.0;
  }
  nav_msgs::GetMap srv;


  while (!map_client.call(srv))
  {
    ros::Duration call_delay(0.5);
    call_delay.sleep();
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Unable to load layout ground truth map. Trying again!");
  }
  ground_truth_map_ = srv.response.map;
  map_client = nh.serviceClient<nav_msgs::GetMap>("/map");
  while (!map_client.call(srv))
  {
    ros::Duration call_delay(0.5);
    call_delay.sleep();
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Unable to load layout map. Trying again!");
  }
  map_ = srv.response.map;

  double factor = map_.info.resolution / ground_truth_map_.info.resolution;
  if (factor == std::floor(factor) && factor >= 1.0)
    compareMaps();
  else
  {
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node","[ipa_map_comparison_node:]"
                                                     "Factor of both maps is not an integer value: "<<factor);
    return;
  }
}

void ipa_map_comparison_node::compareMaps()
{
  ros::NodeHandle nh;
  int factor = map_.info.resolution / ground_truth_map_.info.resolution;
  int width_offset = 0, height_offset = 0, width_left_offset = 0, width_right_offset = 0, height_top_offset = 0,
      height_bottom_offset = 0;
  float occ_count = 0, free_count = 0;
  //check the size of the maps, if they fit into each other without overlapping
  //if they overlap => correct size with offsets
  //necessary for the sliding window comparison later on
  if (factor * map_.info.width > ground_truth_map_.info.width)
  {
    width_offset = factor * map_.info.width - ground_truth_map_.info.width;
    if (width_offset / 2.0 == std::floor((width_offset / 2.0 )))
        width_left_offset = width_right_offset = width_offset/2;
    else
    {
      width_left_offset = std::ceil(width_offset / 2.0);
      width_right_offset = std::floor(width_offset / 2.0);
    }

  }
  if (factor * map_.info.height > ground_truth_map_.info.height)
  {
    height_offset = factor * map_.info.height - ground_truth_map_.info.height;
    if (height_offset / 2.0 == std::floor((width_offset / 2.0 )))
        height_top_offset = height_bottom_offset = height_offset/2;
    else
    {
      height_top_offset = std::ceil(height_offset / 2.0);
      height_bottom_offset = std::floor(height_offset / 2.0);
    }
  }

  //correct size of the ground truth map with offset values and creates 2D array
  double free_score = 0, occ_score = 0, false_free = 0, false_occ = 0;
  std::vector<std::vector<int> > ground_truth_2d_map, map_2d;
  ground_truth_2d_map.resize(ground_truth_map_.info.height + height_offset);
  for (int j = 0; j < ground_truth_map_.info.height + height_offset; j++)
    ground_truth_2d_map[j].resize(ground_truth_map_.info.width + width_offset);
  for (int i = 0; i < ground_truth_map_.info.height + height_offset; i++)
  {
    for (int j = 0; j < ground_truth_map_.info.width + width_offset; j++)
    {
      //cells outside the known area set to unknown
      if (i >= ground_truth_map_.info.height)
      {
        ground_truth_2d_map[i][j] = -1;
      }
      else if (j >= ground_truth_map_.info.width)
        ground_truth_2d_map[i][j] = -1;
      else
      {
        ground_truth_2d_map[i][j] = (ground_truth_map_.data[(i)*ground_truth_map_.info.width + j]);
        if (ground_truth_2d_map[i][j] < -1 || ground_truth_2d_map[i][j] > 100)
          ground_truth_2d_map[i][j]=-1;
        //count number of occupied and free cells in the ref map
        if (ground_truth_map_.data[(i)*ground_truth_map_.info.width + j] >= 50)
          occ_count++;
        else if (ground_truth_map_.data[(i)*ground_truth_map_.info.width + j] >= 0
                 && ground_truth_map_.data[(i)*ground_truth_map_.info.width + j] < 50)
          free_count++;
      }
    }
  }
  map_2d.resize(ground_truth_map_.info.height + height_offset);
  for (int j = 0; j < ground_truth_map_.info.height + height_offset; j++)
    map_2d[j].resize(ground_truth_map_.info.width + width_offset);

  //creates 2D array of measured map
  for (int i = 0; i < (map_.info.height); i++)
  {
    for (int j = 0; j < (map_.info.width); j++)
    {
      for (int inner_i = 0; inner_i < factor; inner_i++)
        for (int inner_j = 0; inner_j < factor; inner_j++)
      {
            map_2d[inner_i + i*factor][inner_j + j*factor]= map_.data[i*map_.info.width + j];
      }
    }
  }

  map_2d_msg_.info.height = map_2d.size();
  map_2d_msg_.info.width = map_2d[0].size();
  map_2d_msg_.info.resolution = ground_truth_map_.info.resolution;
  map_2d_msg_.info.origin = ground_truth_map_.info.origin;

  //create map msgs for debugging
  std::vector<int8_t> map_data((ground_truth_map_.info.width + width_offset) * (ground_truth_map_.info.height + height_offset), -1);
  for (uint y = 0; y < (ground_truth_map_.info.height + height_offset); y++)
    for (uint x = 0; x < (ground_truth_map_.info.width + width_offset); x++)
    {
        map_data.at(y * (ground_truth_map_.info.width + width_offset) + x) = map_2d[y][x];
    }

  map_2d_msg_.data = map_data;
  pub_measured_map_ = nh.advertise<nav_msgs::OccupancyGrid>("map_measured", 1);
  pub_measured_map_.publish(map_2d_msg_);
  ref_2d_msg_.info.height = ground_truth_map_.info.height + height_offset;
  ref_2d_msg_.info.width = ground_truth_map_.info.width + width_offset;
  ref_2d_msg_.info.resolution = ground_truth_map_.info.resolution;
  ref_2d_msg_.info.origin = ground_truth_map_.info.origin;
  std::vector<int8_t> map_data2((ground_truth_map_.info.width + width_offset) * (ground_truth_map_.info.height + height_offset), -1);
  for (uint y = 0; y < (ground_truth_map_.info.height + height_offset); y++)
    for (uint x = 0; x < (ground_truth_map_.info.width + width_offset); x++)
    {
        map_data2.at(y * (ground_truth_map_.info.width + width_offset) + x) = ground_truth_2d_map[y][x];
    }

  ref_2d_msg_.data = map_data2;
  pub_ref_map_ = nh.advertise<nav_msgs::OccupancyGrid>("map_ref", 1);
  pub_ref_map_.publish(ref_2d_msg_);

  //compares both maps according to their values and the given number of neighbours and the neighbourhood_score
  for (int i = 0; i < (ground_truth_map_.info.height + height_offset); i++)
    for (int j = 0; j < (ground_truth_map_.info.width + width_offset); j++)
    {
      int map_occ_value = map_2d[i][j];
      int ground_truth_value = ground_truth_2d_map[i][j];
      if (map_occ_value == ground_truth_value)
      {
        if (map_occ_value >= 50)
          occ_score ++;
        else if(map_occ_value >= 0 && map_occ_value < 50)
          free_score ++;
      }
      else
      {
        if (number_of_neighbours_ == 4)
        {
          if (ground_truth_value == map_2d[i + 1][j] || ground_truth_value == map_2d[i -1][j] ||
              ground_truth_value == map_2d[i][j - 1] || ground_truth_value == map_2d[i][j + 1])
          {
            if (ground_truth_value >= 50)
              occ_score += neighbourhood_score_;
          }
          if (ground_truth_value != -1 && map_occ_value != -1)
          {
            if (map_occ_value >= 50)
              false_occ++;
            else if (map_occ_value >= 0 && map_occ_value < 50)
              false_free++;
          }
        }
        else if(number_of_neighbours_ == 8)
        {
          if (ground_truth_value == map_2d[i + 1][j] || ground_truth_value == map_2d[i -1][j] ||
              ground_truth_value == map_2d[i][j - 1] || ground_truth_value == map_2d[i][j + 1] ||
              ground_truth_value == map_2d[i - 1][j - 1] || ground_truth_value == map_2d[i + 1][j - 1] ||
              ground_truth_value == map_2d[i - 1][j + 1] || ground_truth_value == map_2d[i + 1][j + 1])
          {
            if (ground_truth_value >= 50)
              occ_score += neighbourhood_score_;
          }
          if (ground_truth_value != -1 && map_occ_value != -1)
          {
            if (map_occ_value >= 50)
              false_occ++;
            else if (map_occ_value >= 0 && map_occ_value < 50)
              false_free++;
          }
        }
      }

    }
  free_score /= free_count;
  occ_score /= occ_count;
  false_occ /= free_count;
  false_free /= occ_count;
  ROS_ERROR_STREAM("occ_score: "<<occ_score<< " free_score: "<<free_score<< " false_occ: "<<false_occ<<" false_free: "<<false_free);
  std::ofstream log;
  std::string path = ros::package::getPath("ipa_map_comparison");
  path += "/eval/" + eval_file_name_;
  ROS_INFO_STREAM(path);
  log.open(path.c_str(), std::ofstream::out | std::ofstream::app);
  log<<"occ_score: "<<occ_score<<std::endl;
  log<<"free_score: "<<free_score<<std::endl;
  log<<"false_occ: "<<false_occ<<std::endl;
  log<<"false_free: "<<false_free<<std::endl;
  log.close();
}
void ipa_map_comparison_node::publish()
{
  pub_measured_map_.publish(map_2d_msg_);
  pub_ref_map_.publish(ref_2d_msg_);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ipa_map_comparison_node");
  ros::NodeHandle n;

  // wait till first clock message is received
  while (ros::Time::now().sec == 0)
  {
  }
  ros::Duration(3).sleep();
  ipa_map_comparison_node comp_node = ipa_map_comparison_node();
  while (ros::ok())
  {
    comp_node.publish();
    ros::spinOnce();
  }
  return 0;
}
