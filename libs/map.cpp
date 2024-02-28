#include "map.h"


MapConfig extract_map(const YAML::Node& config) {
    MapConfig mapconfig;

        if (config["map"]){ 
            mapconfig.width = config["map"]["width"].as<double>();
            mapconfig.type = config["map"]["type"].as<std::string>();
            mapconfig.height = config["map"]["height"].as<double>();
            mapconfig.resolution = config["map"]["resolution"].as<double>();
            mapconfig.color.r=255*config["map"]["color"]["r"].as<double>();
            mapconfig.color.g=255*config["map"]["color"]["g"].as<double>();
            mapconfig.color.b=255*config["map"]["color"]["b"].as<double>();
            
        }
        else {
            // Print debug information
            std::cout << "YAML Exception while extracting map: " << std::endl;
            // Rethrow the exception to terminate the program
            throw;
        }

    return mapconfig;
}


std::vector<Obstacle> extract_obs(const YAML::Node& config) {

	std::vector<Obstacle> obstacles;
	if (config["obstacles"]){
	for (const auto& obstacleNode : config["obstacles"]) {
	Obstacle obstacle;
	obstacle.name = obstacleNode["name"].as<std::string>();
	obstacle.type = obstacleNode["type"].as<std::string>();
	obstacle.center.x = obstacleNode["center"]["x"].as<double>();
	obstacle.center.y = obstacleNode["center"]["y"].as<double>();
	obstacle.center.theta = obstacleNode["center"]["theta"].as<double>();
	obstacle.size.width = obstacleNode["size"]["width"].as<double>();
	obstacle.size.length = obstacleNode["size"]["length"].as<double>();
	obstacle.size.radius = obstacleNode["size"]["radius"].as<double>();
	obstacles.push_back(obstacle);
	}
	}
	else {
            std::cout << "YAML Exception while extracting obstacles: " << std::endl;
            throw;
        }
	return obstacles;
}
