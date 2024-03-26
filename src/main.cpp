#include "robots.h"
#include "map.h"
#include "lidar.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "visualize.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
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
int intensity_threshold = 230;
std::vector<sensor_msgs::LaserScan> lidar_scans;

bool is_mouse_dragging = false;
std::vector<bool> is_robot_dragging(robots.size(), false);
double zoomFactor = 1.0;
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
	else if (event == cv::EVENT_MOUSEWHEEL) {
	// Mouse wheel event for zooming
	if (flags > 0) {
	    // Zoom in
	    zoomFactor *= 1.2;  // You can adjust the zoom factor
	} else {
	    // Zoom out
	    zoomFactor *= 0.8;  // You can adjust the zoom factor
	}

	// Resize the displayed image according to the zoom factor
	cv::resizeWindow("multi_xy_robot_simulator", static_cast<int>(map.width * zoomFactor), static_cast<int>(map.height * zoomFactor));
	}
	}


geometry_msgs::Twist velocity;

std::vector<geometry_msgs::Twist> velocities(robots.size());

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
    velocity.linear.x = cmd_vel_msg->linear.x;
    velocity.linear.y = cmd_vel_msg->linear.y;
    velocity.angular.z = cmd_vel_msg->angular.z;
    
}

std::string robotName;


void goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

tf::Quaternion quat;
tf::quaternionMsgToTF(msg->pose.orientation, quat);
// Convert quaternion to yaw (theta)
double theta = tf::getYaw(quat);

cv::Point2d center =worldToImage({msg->pose.position.x ,msg->pose.position.y,0}, map);
mapImage1 = drawArrow(mapImage1, center, theta, (0,0,0),5);
}



int main(int argc, char **argv) {

        if (argc != 2) {
        std::cerr << "Usage: " << "rosrun xy_simulator main"<< " <robot_name>" << std::endl;
        robotName = "Bot1"; // Setting  default robot name
        }
        else{
          robotName= argv[1];  
        }    
        
        
        robotName = "Bot1";
        std::cout<<"Selected robot for navigation stack : "<< robotName<< std::endl;
	mapImage = cv::imread(map.path); 
	map.width=mapImage.cols*map.resolution;
	map.height = mapImage.rows*map.resolution;
	std::cout<<"rows: "<<mapImage.rows<<"cols: "<<mapImage.cols<<std::endl;
	cv::namedWindow("multi_xy_robot_simulator");
	cv::setMouseCallback("multi_xy_robot_simulator", onMouse);
	
	ros::init(argc, argv, "xy_simulator");
	ros::NodeHandle nh;
	ros::Rate r(10);
	std::vector<ros::Publisher> pose_publishers;
	std::vector<ros::Publisher> lidar_publishers;
	tf::TransformBroadcaster tf_broadcaster;
	std::vector<ros::Publisher> odom_publishers; 
	

	
	
	for (size_t r=0; r<robots.size();++r) {
        pose_publishers.push_back(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "/"+robots[r].name+"_pose", 10));
        lidar_publishers.push_back(nh.advertise<sensor_msgs::LaserScan>("/"+robots[r].name+"_scan", 10)); 
        odom_publishers.push_back (nh.advertise<nav_msgs::Odometry>("/"+robots[r].name+"_odom", 10)); 

        }
        
        ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
        
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

        
	while (ros::ok()) {
		mapImage1 = mapImage.clone();       
		cv::cvtColor(mapImage1, gray_image, cv::COLOR_BGR2GRAY);

		mapImage1 = add_robots(mapImage1, map, robots);
		mapImage1=Lidar(mapImage1, gray_image, map, robots, intensity_threshold,lidar_publishers);
		 
		
		ros::Subscriber goal = nh.subscribe("/move_base_simple/goal", 10, goal_Callback);
		cv::imshow("multi_xy_robot_simulator", mapImage1);
		int key = cv::waitKey(1);
		

		
		geometry_msgs::TransformStamped odom0_tf;
		odom0_tf.header.frame_id = "map";  // 
		odom0_tf.transform.translation.x = 0;
		odom0_tf.transform.translation.y = 0;
		odom0_tf.transform.translation.z = 0.0;
		odom0_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);
		odom0_tf.header.stamp = ros::Time::now();
		odom0_tf.child_frame_id = "odom";
		tf_broadcaster.sendTransform(odom0_tf); 
		
		
		
		
		for (size_t p=0; p<robots.size();++p) 
		{
		
		
		// Publishing Pose in ROS
		
		geometry_msgs::PoseWithCovarianceStamped pose_msg;
		pose_msg.header.stamp = ros::Time::now();
		pose_msg.header.frame_id = robots[p].frame_id;
		pose_msg.pose.pose.position.x = robots[p].pose.x; 
		pose_msg.pose.pose.position.y = robots[p].pose.y; 
		pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robots[p].pose.theta); 
		pose_msg.pose.covariance = {0};
		pose_publishers[p].publish(pose_msg); 
		
		
		// Publishing tf 
		
		
		geometry_msgs::TransformStamped pose_data;
		pose_data.header.frame_id = robots[p].frame_id;  //setting parent frame from yaml
		pose_data.transform.translation.x = robots[p].pose.x;
		pose_data.transform.translation.y = robots[p].pose.y;
		pose_data.transform.translation.z = 0.0;
		pose_data.transform.rotation = tf::createQuaternionMsgFromYaw(robots[p].pose.theta);
		pose_data.header.stamp = ros::Time::now();
		pose_data.child_frame_id = robots[p].name +"_baselink";  
		tf_broadcaster.sendTransform(pose_data);
		
		
		geometry_msgs::TransformStamped sensor_tf;
		sensor_tf.header.frame_id = robots[p].name+ "_baselink";  // 
		sensor_tf.transform.translation.x = 0;
		sensor_tf.transform.translation.y = 0;
		sensor_tf.transform.translation.z = 0.0;
		sensor_tf.transform.rotation = tf::createQuaternionMsgFromYaw(-robots[p].pose.theta); //lidar is always at 0 degree with robot
		sensor_tf.header.stamp = ros::Time::now();
		sensor_tf.child_frame_id = robots[p].name+ "_scan_frame";  
		tf_broadcaster.sendTransform(sensor_tf);
		
		geometry_msgs::TransformStamped odom_tf;
		odom_tf.header.frame_id = "odom";  
		odom_tf.transform.translation.x = robots[p].pose.x;
		odom_tf.transform.translation.y = robots[p].pose.y;
		odom_tf.transform.translation.z = 0.0;
		odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(robots[p].pose.theta);
		odom_tf.header.stamp = ros::Time::now();
		odom_tf.child_frame_id = robots[p].name+ "_baselink";
		tf_broadcaster.sendTransform(odom_tf);
		
		
		
		
		
		
		if (robotName== robots[p].name){
		
		velocities[p]=  velocity;
		

		}
		
		// Assuming  a fixed time step for updating the robot's pose
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		//dt=0.1;

		// Update robot's position based on velocities
		robots[p].pose.theta += velocities[p].angular.z * dt;
		robots[p].pose.x += (velocities[p].linear.x* cos( robots[p].pose.theta) * dt);
		robots[p].pose.y += (velocities[p].linear.x* sin( robots[p].pose.theta) * dt);
		


		// Ensure orientation is in the range [0, 2*pi)
		if (robots[p].pose.theta >= 2.0 * M_PI) {
		robots[p].pose.theta -= 2.0 * M_PI;
		} else if (robots[p].pose.theta < 0.0) {
		robots[p].pose.theta += 2.0 * M_PI;
		}
		
		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "map";
		odom.pose.pose.position.x = robots[p].pose.x;
		odom.pose.pose.position.y = robots[p].pose.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robots[p].pose.theta);
		odom.child_frame_id = robots[p].name+ "_baselink";
		
		odom.twist.twist = velocities[p];

		//publish the message
		odom_publishers[p].publish(odom);
		
		
		
		}
		
		
		
		last_time = current_time;
		ros::spinOnce();

	}


	return 0;
	}

