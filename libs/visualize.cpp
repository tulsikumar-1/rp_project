#include "visualize.h"
#include <iostream>



cv::Point2d worldToImage(const Pose& pose, const MapConfig& mapConfig) {
    int x, y;

        x = static_cast<int>((pose.x + mapConfig.width / 2) / mapConfig.resolution);
        y = static_cast<int>((-pose.y + mapConfig.height / 2) / mapConfig.resolution);

    return cv::Point2d(x, y);
}


Pose imageToWorld(const cv::Point2d& image_point, const MapConfig& mapConfig) {
    double x, y;
    
    x = (image_point.x * mapConfig.resolution) - mapConfig.width / 2;
    y = -(image_point.y * mapConfig.resolution) + mapConfig.height / 2;
    
    return {x,y,0}; 
}


cv::Mat drawArrow(cv::Mat& image, const cv::Point2i& start, double angle, const cv::Scalar& color, double size) { 
    cv::Point2i end(start.x + size * std::cos(-angle), start.y + size * std::sin(-angle));
    cv::arrowedLine(image, start, end, color, 2);
    return image;  
}


// Draw the robots on the map

cv::Mat add_robots(cv::Mat& mapImage ,const MapConfig& mapConfig, const std::vector<Robot>& robots){

	for (const auto& robot : robots) {
	cv::Scalar color_a(0,0,0, 255);
	cv::Scalar color_r(robot.color.b * 255, robot.color.g * 255, robot.color.r * 255,robot.color.a * 128);

	int radius =robot.dimensions.radius/mapConfig.resolution;
	cv::Point2d center =worldToImage(robot.pose, mapConfig);
	
	//std::cout<<robot.pose.x<< " robot  "<< robot.pose.y<<std::endl;
	cv::circle(mapImage, center, radius, color_r, cv::FILLED);
	cv::circle(mapImage, center, radius, cv::Scalar(0,0,0,255),1);
	mapImage = drawArrow(mapImage, center, robot.pose.theta, color_a,radius-2);

	//cv::putText(mapImage, robot.name, worldToImage(robot.pose, mapConfig) + cv::Point2d(15, -15),
	//cv::FONT_HERSHEY_SIMPLEX, 0.5, color_r, 1);
	}

	return  mapImage;         
}            
            


