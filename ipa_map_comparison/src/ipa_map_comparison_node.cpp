#include "ipa_map_comparison/ipa_map_comparison_node.h"

ipa_map_comparison_node::ipa_map_comparison_node()
{
  map_eval_started_ = false;
  ros::NodeHandle nh("~");
  ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
  neighbourhood_score_ = 0.0;
  number_of_neighbours_ = 0;
  eval_file_name_ = "test.txt";
  double min_free_score = 0.9, min_occ_score = 0.9;
  std::string old_map_filepath, new_map_filepath;
  int maps_size;
  nh.getParam("neighbourhood_score",neighbourhood_score_);
  nh.getParam("number_of_neighbours",number_of_neighbours_);
  nh.getParam("eval_file_name", eval_file_name_);
  nh.getParam("min_occ_score", min_occ_score);
  nh.getParam("min_free_score", min_free_score);
  nh.getParam("old_map", old_map_filepath);
  nh.getParam("new_map", new_map_filepath);
  nh.getParam("submap_size", maps_size);

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
//  nav_msgs::GetMap srv;


//  while (!map_client.call(srv))
//  {
//    ros::Duration call_delay(0.5);
//    call_delay.sleep();
//    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Unable to load layout ground truth map. Trying again!");
//  }
  // => /home/fmw-hb/.ipa_navigation/long_term_slam_backup/client/backup_new.backu
  reference_map_ = readBackupMap(old_map_filepath);
//  map_client = nh.serviceClient<nav_msgs::GetMap>("/map");
//  while (!map_client.call(srv))
//  {
//    ros::Duration call_delay(0.5);
//    call_delay.sleep();
//    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Unable to load layout map. Trying again!");
//  }
  // => /home/fmw-hb/.ipa_navigation/long_term_slam_backup/client/long_term_slam.backup
 compare_map_ = readBackupMap(new_map_filepath);

 if(reference_map_.data.size()>0 && compare_map_.data.size()>0)
 {
 ref_parts_ = splitMap(reference_map_, maps_size);
 comp_parts_ = splitMap(compare_map_, maps_size);
 }
 std::vector<std::map<std::string, double>> results;
 double factor = compare_map_.info.resolution / reference_map_.info.resolution;
  if (factor == std::floor(factor) && factor >= 1.0 && ref_parts_.size() == comp_parts_.size())
  {
   for (int i = 0; i < ref_parts_.size(); i++)
   {
      results.push_back(compareMaps(ref_parts_[i], comp_parts_[i]));
   }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node","[ipa_map_comparison_node:]"
                                                     "Factor of both maps is not an integer value: "<<factor);
    return;
  }
  //writeLogFile(results);

  if (min_occ_score < 0 || min_occ_score > 1.0 || min_free_score < 0 || min_occ_score > 1.0)
  {
    ROS_ERROR_STREAM("Minimal occupancy and free score must be between 0 and 1, are "<<min_occ_score<<" and "<<min_free_score);
    return;
  }

  bool all_parts_sane = true;
  for (std::vector<std::map<std::string, double>>::iterator it = results.begin(); it != results.end(); ++it)
    all_parts_sane = it->at("occ_score") > min_occ_score && it->at("free_score") > min_free_score && all_parts_sane;

  if (all_parts_sane)
  {
    ROS_INFO("Map pausible - updating backup...");
    //replaceBackup(old_map_filepath, new_map_filepath);
  }
  else
    ROS_WARN("Scores too low - do not update backup!");
}

std::map<std::string, double> ipa_map_comparison_node::compareMaps(nav_msgs::OccupancyGrid& reference_map, nav_msgs::OccupancyGrid& compare_map)
{
  ros::NodeHandle nh;
  int factor =compare_map.info.resolution / reference_map.info.resolution;
  int width_offset = 0, height_offset = 0, width_left_offset = 0, width_right_offset = 0, height_top_offset = 0,
      height_bottom_offset = 0;
  float occ_count = 0, free_count = 0;
  //check the size of the maps, if they fit into each other without overlapping
  //if they overlap => correct size with offsets
  //necessary for the sliding window comparison later on
  if (factor *compare_map.info.width > reference_map.info.width)
  {
    width_offset = factor *compare_map.info.width - reference_map.info.width;
    if (width_offset / 2.0 == std::floor((width_offset / 2.0 )))
        width_left_offset = width_right_offset = width_offset/2;
    else
    {
      width_left_offset = std::ceil(width_offset / 2.0);
      width_right_offset = std::floor(width_offset / 2.0);
    }

  }
  if (factor *compare_map.info.height > reference_map.info.height)
  {
    height_offset = factor *compare_map.info.height - reference_map.info.height;
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
  ground_truth_2d_map.resize(reference_map.info.height + height_offset);
  for (int j = 0; j < reference_map.info.height + height_offset; j++)
    ground_truth_2d_map[j].resize(reference_map.info.width + width_offset);
  for (int i = 0; i < reference_map.info.height + height_offset; i++)
  {
    for (int j = 0; j < reference_map.info.width + width_offset; j++)
    {
      //cells outside the known area set to unknown
      if (i >= reference_map.info.height)
      {
        ground_truth_2d_map[i][j] = -1;
      }
      else if (j >= reference_map.info.width)
        ground_truth_2d_map[i][j] = -1;
      else
      {
        ground_truth_2d_map[i][j] = (reference_map.data[(i)*reference_map.info.width + j]);
        if (ground_truth_2d_map[i][j] < -1 || ground_truth_2d_map[i][j] > 100)
          ground_truth_2d_map[i][j]=-1;
        //count number of occupied and free cells in the ref map
        if (reference_map.data[(i)*reference_map.info.width + j] >= 50)
          occ_count++;
        else if (reference_map.data[(i)*reference_map.info.width + j] >= 0
                 && reference_map.data[(i)*reference_map.info.width + j] < 50)
          free_count++;
      }
    }
  }
  map_2d.resize(reference_map.info.height + height_offset);
  for (int j = 0; j < reference_map.info.height + height_offset; j++)
    map_2d[j].resize(reference_map.info.width + width_offset);

  //creates 2D array of measured map
  for (int i = 0; i < (compare_map.info.height); i++)
  {
    for (int j = 0; j < (compare_map.info.width); j++)
    {
      for (int inner_i = 0; inner_i < factor; inner_i++)
        for (int inner_j = 0; inner_j < factor; inner_j++)
      {
            map_2d[inner_i + i*factor][inner_j + j*factor]=compare_map.data[i*compare_map.info.width + j];
      }
    }
  }

  map_2d_msg_.info.height = map_2d.size();
  map_2d_msg_.info.width = map_2d[0].size();
  map_2d_msg_.info.resolution = reference_map.info.resolution;
  map_2d_msg_.info.origin = reference_map.info.origin;

  //create map msgs for debugging
  std::vector<int8_t> map_data((reference_map.info.width + width_offset) * (reference_map.info.height + height_offset), -1);
  for (uint y = 0; y < (reference_map.info.height + height_offset); y++)
    for (uint x = 0; x < (reference_map.info.width + width_offset); x++)
    {
        map_data.at(y * (reference_map.info.width + width_offset) + x) = map_2d[y][x];
    }

  map_2d_msg_.data = map_data;
  pub_measured_map_ = nh.advertise<nav_msgs::OccupancyGrid>("map_measured", 1);
  pub_measured_map_.publish(map_2d_msg_);
  ref_2d_msg_.info.height = reference_map.info.height + height_offset;
  ref_2d_msg_.info.width = reference_map.info.width + width_offset;
  ref_2d_msg_.info.resolution = reference_map.info.resolution;
  ref_2d_msg_.info.origin = reference_map.info.origin;
  std::vector<int8_t> map_data2((reference_map.info.width + width_offset) * (reference_map.info.height + height_offset), -1);
  for (uint y = 0; y < (reference_map.info.height + height_offset); y++)
    for (uint x = 0; x < (reference_map.info.width + width_offset); x++)
    {
        map_data2.at(y * (reference_map.info.width + width_offset) + x) = ground_truth_2d_map[y][x];
    }

  ref_2d_msg_.data = map_data2;
  pub_ref_map_ = nh.advertise<nav_msgs::OccupancyGrid>("map_ref", 1);
  pub_ref_map_.publish(ref_2d_msg_);

  //compares both maps according to their values and the given number of neighbours and the neighbourhood_score
  for (int i = 0; i < (reference_map.info.height + height_offset); i++)
    for (int j = 0; j < (reference_map.info.width + width_offset); j++)
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
  std::cout<<"occ_score: "<<occ_score<< " free_score: "<<free_score<< " false_occ: "<<false_occ<<" false_free: "<<false_free<<std::endl;

  std::map<std::string, double> results;
  results.insert(std::pair<std::string, double>("occ_score",occ_score));
  results.insert(std::pair<std::string, double>("free_score",free_score));
  results.insert(std::pair<std::string, double>("false_occ",false_occ));
  results.insert(std::pair<std::string, double>("false_free",false_free));
  return results;
}

std::vector<nav_msgs::OccupancyGrid> ipa_map_comparison_node::splitMap(nav_msgs::OccupancyGrid local_map, int size)
{
  std::string pub_frame = "split_map";
  //prepare it for the publish
    ROS_INFO("Got Map to split");
    ROS_INFO("Publish meta_data_packet");
    nav_msgs::OccupancyGrid * meta_data = new nav_msgs::OccupancyGrid();
    meta_data->header.frame_id = pub_frame;
    meta_data->info.height = local_map.info.height;
    meta_data->info.width = local_map.info.width;
    meta_data->info.resolution = local_map.info.resolution;

    ros::Duration(3).sleep();
    std::vector<nav_msgs::OccupancyGrid> map_parts;
    for(int row = 0; row < local_map.info.height-size;row+=size)
    {
        for(int collum = 0; collum < local_map.info.width-size;collum+=size)
        {
          nav_msgs::OccupancyGrid* map_part = getMapPart(&local_map,row,collum,size,size);
          map_parts.push_back(*map_part);
        }

    }
    return map_parts;
}

nav_msgs::OccupancyGrid* ipa_map_comparison_node::getMapPart(nav_msgs::OccupancyGrid *tmp, int start_x, int start_y, int width, int height)
{
    nav_msgs::OccupancyGrid* part = new nav_msgs::OccupancyGrid();
    if (tmp->data.size() < 1)
    {
      ROS_ERROR_STREAM("map empty (size: "<<tmp->data.size()<<")");
      return part;
    }
    for(int row = 0; row < height;row ++)
    {
        for(int collum = 0; collum < width;collum++)
           part->data.push_back(tmp->data.at((row+start_x)*tmp->info.width+(collum + start_y)));
    }
    part->header = tmp->header;
    part->info.origin.position.x = start_x;
    part->info.origin.position.y = start_y;
    part->info.height = height;
    part->info.width = width;
    part->info.resolution = tmp->info.resolution;
    return part;
}

void ipa_map_comparison_node::writeLogFile(std::map<std::string, double> results)
{
  std::ofstream log;
  std::string path = ros::package::getPath("ipa_map_comparison");
  path += "/eval/" + eval_file_name_;
  ROS_INFO_STREAM("Logfile output path: "<<path);
  log.open(path.c_str(), std::ofstream::out | std::ofstream::app);
  log<<"time: "<<ros::Time::now()<<std::endl;
  log<<"occ_score: "<<results.at("occ_score")<<std::endl;
  log<<"free_score: "<<results.at("free_score")<<std::endl;
  log<<"false_occ: "<<results.at("false_occ")<<std::endl;
  log<<"false_free: "<<results.at("false_free")<<std::endl;
  log.close();
}

bool ipa_map_comparison_node::replaceBackup(std::string old_map, std::string new_map)
{
  //from http://stackoverflow.com/questions/10195343/copy-a-file-in-a-sane-safe-and-efficient-way
  int source = open(old_map.c_str(), O_RDONLY, 0);
  int dest = open(new_map.c_str(), O_WRONLY | O_CREAT /*| O_TRUNC/**/, 0644);

  // struct required, rationale: function stat() exists also
  struct stat stat_source;
  fstat(source, &stat_source);

  sendfile(dest, source, 0, stat_source.st_size);

  close(source);
  close(dest);
  ROS_INFO("Backup Replaced!!!");
}

nav_msgs::OccupancyGrid ipa_map_comparison_node::readBackupMap(std::string filepath)
{

  nav_msgs::OccupancyGrid map;
  /*ipa_navigation_msgs::HmmSlamBackup backup_msg;

  // load file from disc
  std::string backup_filepath = std::getenv("HOME");
  backup_filepath += "/.ipa_navigation/long_term_slam_backup/client";
  ROS_INFO_STREAM(backup_filepath+"/"+filepath);
  if (!boost::filesystem::exists(backup_filepath+"/"+filepath))
  {
    ROS_ERROR_STREAM("Could not find file:" << filepath);
    return map;
  }
  std::ifstream ifs(backup_filepath+"/"+filepath, std::ios::in | std::ios::binary);

  if (!ifs.is_open())
  {
    ROS_ERROR_STREAM("Could not open file:"<< backup_filepath<<"/"<<filepath);
    return map;
  }

  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  uint32_t file_size = end - begin;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read((char*)ibuffer.get(), file_size);
  ros::serialization::IStream istream(ibuffer.get(), file_size);
  ros::serialization::deserialize(istream, backup_msg);
  ifs.close();

  ROS_INFO_STREAM_NAMED("SlamManager.initFromBackup", "[SlamManager.initFromBackup]: Successfully read in backup file");

  // Initializing reference map
  map.info.resolution = backup_msg.longterm_map.resolution;
  map.info.width = backup_msg.longterm_map.width;
  map.info.height = backup_msg.longterm_map.height;
  std::vector<int8_t> grid_vector(map.info.width*map.info.height, 0 );
  for (uint y = 0; y < map.info.height; y++)
  {
    for (uint x = 0; x < map.info.width; x++)
    {
      float occ_value = backup_msg.longterm_map.hmmCellMat[x].hmmCellArray[y].occ * 100;
      uint cell_position = y  * map.info.width + x;
      //std::cout<<"occ value: "<<occ_value<<" at "<<x<<", "<<y<<", cellpos: "<<cell_position<<std::endl;
      if (occ_value > 50)
        grid_vector.at(cell_position) = 100;
      else if (occ_value < 0)
        grid_vector.at(cell_position) = -1;
    }
  }
  map.data = grid_vector;

  ROS_INFO_STREAM("Loaded map from backup \nfilepath: "<<backup_filepath<<"/"<<filepath);
*/
  return map;
}

void ipa_map_comparison_node::publish()
{
  pub_measured_map_.publish(map_2d_msg_);
  pub_ref_map_.publish(ref_2d_msg_);
  ros::Duration(1).sleep();
  for (int i = 0; i < ref_parts_.size(); i++)
  {
    pub_ref_map_.publish(ref_parts_[i]);
    pub_measured_map_.publish(comp_parts_[i]);
    ros::Duration(3).sleep();
  }
//  for( std::vector<nav_msgs::OccupancyGrid>::iterator it = ref_parts_.begin(); it != ref_parts_.end(); ++it)
//  {
//    pub_ref_map_.publish(*it);
//  }
//  for( std::vector<nav_msgs::OccupancyGrid>::iterator it = comp_parts_.begin(); it != comp_parts_.end(); ++it)
//  {
//    pub_measured_map_.publish(*it);
//  }
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
    ros::Duration(3).sleep();
    ros::spinOnce();
  }
  return 0;
}
