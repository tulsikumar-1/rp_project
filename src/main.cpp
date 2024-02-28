#include "robots.h"
#include "map.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "visualize.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>


YAML::Node config = YAML::LoadFile("/home/tulsi/rp_project/src/xy_simulator/config/config.yaml");

MapConfig map = extract_map(config);
std::vector<Robot> robots = extractRobots(config);


cv::Mat mapImage;
cv::Mat mapImage1;
cv::Mat gray_image;


bool is_mouse_dragging = false;
std::vector<bool> is_robot_dragging(robots.size(), false);

void onMouse(int event, int x, int y, int flags, void* param) {
 
	if (event == cv::EVENT_LBUTTONDOWN) {
		for (size_t i = 0; i < robots.size(); ++i) {
		cv::Point2d robot_position = worldToImage(robots[i].pose, map);
		double distance = std::sqrt(std::pow(x - robot_position.x, 2) + std::pow(y - robot_position.y, 2));
			if (distance <= robots[i].dimensions.radius / map.resolution) {
			is_robot_dragging[i] = true;
			//std::cout << "Called for Robot " << i << std::endl;
			}}} 
			
	else if (event == cv::EVENT_MOUSEMOVE) {
		for (size_t i = 0; i < robots.size(); ++i) {
			if (is_robot_dragging[i]) {
			
			robots[i].pose = imageToWorld(cv::Point2d(x,y),map);

			// Ensure the robot stays within the map boundaries
			robots[i].pose.x = std::max(robots[i].pose.x, -(0.5*map.width ));
			robots[i].pose.y = std::max(robots[i].pose.y, -(0.5*map.height ));
			robots[i].pose.x = std::min(robots[i].pose.x, (0.5*map.width ));
			robots[i].pose.y = std::min(robots[i].pose.y, (0.5*map.height ));
			//std::cout << "No Collision for Robot " << i << std::endl;
			}}} 
			
	else if (event == cv::EVENT_LBUTTONUP) {
		for (size_t i = 0; i < robots.size(); ++i) {
		is_robot_dragging[i] = false;
		//std::cout << "Released for Robot " << i << std::endl;
		}}
}



int main(int argc, char **argv) {
               std::cout << "map_path " <<config["map"]["path"].as<std::string>() << std::endl;
		mapImage = cv::imread(map.path); // Provide the path
		map.width=mapImage.cols*map.resolution;
		map.height = mapImage.rows*map.resolution;

	ros::init(argc, argv, "xy_simulator");
	
	cv::namedWindow("multi_xy_robot_simulator");
	cv::setMouseCallback("multi_xy_robot_simulator", onMouse);

    
    while (ros::ok()) {
        mapImage1 = mapImage.clone();
        
        cv::cvtColor(mapImage1, gray_image, cv::COLOR_BGR2GRAY);
        
        		
		//std::cout << "map width " <<map.width << std::endl;
		//std::cout << "map Height " <<map.height << std::endl;

        mapImage1 = add_robots(mapImage1, map, robots);
        cv::imshow("multi_xy_robot_simulator", mapImage1);
        cv::waitKey(10);
        ros::spinOnce();
        
       }
       
       
    return 0;
}

