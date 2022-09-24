#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace grid_map;

int main (int argc, char ** argv) {
  
  ros::init(argc, argv, "image2map");
  ros::NodeHandle node("~");

  std::string elevation_topic,
              temperature_topic,
              map_topic,
							map_frame;

	float elevation_min, elevation_max,
				temperature_min, temperature_max;

	float resolution;
	float center_x, center_y;

  node.getParam("elevation_image_topic", elevation_topic);
	node.getParam("elevation_min_value", elevation_min);
	node.getParam("elevation_max_value", elevation_max);
	node.getParam("temperature_min_value", temperature_min);
	node.getParam("temperature_max_value", temperature_max);
  node.getParam("temperature_image_topic", temperature_topic);
  node.getParam("map_topic", map_topic);
	node.getParam("map_frame_id", map_frame);
	node.getParam("center_x", center_x);
	node.getParam("center_y", center_y);

  GridMap map;

	bool init = false;

  ros::Publisher map_pub = node.advertise<grid_map_msgs::GridMap>(map_topic, 10, true);

  auto elevCallback = [&](const sensor_msgs::ImageConstPtr &msg_in) {
		
		if ((!init)) {
    	if (GridMapRosConverter::initializeFromImage(*msg_in, resolution, map, Position(center_x, center_y)))
				ROS_INFO("Map initialized.");
			else
				ROS_INFO("Error on map initialization.");
		}

		if(GridMapRosConverter::addLayerFromImage(*msg_in, "elevation", map, elevation_min, elevation_max))
			ROS_INFO("Elevation layer added to map.");
		else
			ROS_INFO("Error adding elevation layer to map.");

		init = true;
  };

	auto tempCallback = [&](const sensor_msgs::ImageConstPtr &msg_in) {
    if (GridMapRosConverter::addLayerFromImage(*msg_in, "temperature", map, elevation_min, elevation_max))
			ROS_INFO("Temperature layer added to map.");
		else
			ROS_INFO("Error adding temperature layer to map.");
  };
  
  ros::Subscriber elev_sub = node.subscribe<sensor_msgs::Image>(elevation_topic, 10, elevCallback);
	ros::Subscriber temp_sub = node.subscribe<sensor_msgs::Image>(elevation_topic, 10, tempCallback);

  ros::Rate(10);
	while (node.ok()) {

		if (init) {
			grid_map_msgs::GridMap map_out;
			GridMapRosConverter::toMessage(map, map_out);
			map_out.info.header.stamp = ros::Time::now();
			map_out.info.header.frame_id = map_frame;
			map_pub.publish(map_out);
		}

		ros::spinOnce();
	}
  
  return 0; 
}