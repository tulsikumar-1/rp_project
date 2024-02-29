#include "lidar.h"


cv::Mat Lidar(cv::Mat& mapImage ,cv::Mat& gray_image,const MapConfig& mapConfig,  
std::vector<Robot>& robots,const int intensity_threshold,std::vector<ros::Publisher> lidar_publisher){



//for (size_t r=0; r<robots.size();++r)
for (size_t r=0; r<1;++r)
{
        Robot robot = robots[r];
	int num_beams = robot.lidar.num_beams;     // Number of LiDAR beams
	double min_range =robot.lidar.min_range+robot.dimensions.radius;   // Minimum LiDAR range
	double max_range = robot.lidar.max_range;   // Maximum LiDAR range
	cv::Scalar color(robot.color.b * 255, robot.color.g * 255, robot.color.r * 255,robot.color.a * 128);

	std::vector<Robot> robotsCopy = robots;
	robotsCopy.erase(std::remove_if(robotsCopy.begin(), robotsCopy.end(), [&robot](const Robot& robot1) { return robot1.name == robot.name; }), robotsCopy.end());
	
	gray_image = add_robots(gray_image, mapConfig,robotsCopy);
	sensor_msgs::LaserScan lidar_data;
	lidar_data.header.stamp = ros::Time::now();
	lidar_data.header.frame_id = "scan_frame";  // to be changed
	
	lidar_data.angle_min =  0;  
	lidar_data.angle_max = 2*M_PI; 
	lidar_data.angle_increment = (2*M_PI)/(num_beams);
	lidar_data.time_increment = 0.1/num_beams;   
	lidar_data.scan_time = 0.1;       
	lidar_data.range_min = min_range;     
	lidar_data.range_max = max_range;
	lidar_data.ranges=std::vector<float>(num_beams, 0.0);
        

for (int i = 0; i <num_beams; ++i) {
    
    double angle = i * (2.0 * M_PI / num_beams);  // Angle in radians
    
    
    
    for (double range = min_range; range <= max_range; range += 0.1) {
        // Convert polar coordinates to Cartesian coordinates
                
        cv::Point2d map = worldToImage({robot.pose.x + range * cos(angle) ,robot.pose.y + range*sin(angle),0}, mapConfig);
        
       // std::cout<< i<<"  " <<map.x<<" ,  "<< map.y<<std::endl;
        // Check if the point is inside the 2D map bounds
        if (map.x >= 0 && map.x < gray_image.rows && map.y >= 0 && map.y < gray_image.cols) {
            // 
            if (gray_image.at<uchar>(map.y, map.x) < intensity_threshold) {

                lidar_data.ranges[i]=range ;
                        
                cv::circle(mapImage, cv::Point(map.x, map.y), 2, color, -1);
             //   std::cout<<"drawn"; 
                break; // Break after detecting boundary point
            }
            
           
            
        }
    }
}       
        
       // std::cout<<"np prob in loop";
        lidar_publisher[r].publish(lidar_data);
        

}
 return mapImage;

}
