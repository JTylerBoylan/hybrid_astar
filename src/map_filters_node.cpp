#include <ros/ros.h>

#include <filters/filter_chain.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace grid_map;

int main (int argc, char ** argv) {
  
  ros::init(argc, argv, "map_filters");
  ros::NodeHandle node("~");
  
  filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
  if (!filterChain.configure("grid_map_filters", node)) {
    ROS_ERROR("Could not configure the filter chain!");
    return 0;
  }

  std::string input_topic,
              output_topic;

  node.getParam("input_topic", input_topic);
  node.getParam("output_topic", output_topic);

  GridMap map;

  ros::Publisher map_pub = node.advertise<grid_map_msgs::GridMap>(output_topic, 10, true);

  auto mapCallback = [&](const grid_map_msgs::GridMapConstPtr &msg_in) {

    GridMap map_in, map_out;

    GridMapRosConverter::fromMessage(*msg_in, map_in);

    if (!filterChain.update(map, map_out)) {
      ROS_ERROR("Could not update the grid map filter chain!");
      return;
    }

    grid_map_msgs::GridMap msg_out;
    GridMapRosConverter::toMessage(map_out, msg_out);

    map_pub.publish(msg_out);
    
  };
  
  ros::Subscriber map_sub = node.subscribe<grid_map_msgs::GridMap>(input_topic, 10, mapCallback);

  ros::spin();
  
  return 0; 
}
