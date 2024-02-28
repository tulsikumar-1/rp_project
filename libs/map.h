#ifndef map_H
#define map_H


#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <iostream>
#include "robots.h"



  
struct MapConfig {
    std::string type;
    double width;
    double height;
    double resolution;
    Color color;
 
    
} ; 

struct Obstacle {
    std::string name;
    std::string type;
    Pose center;
    Dimensions size;
    
};

MapConfig extract_map(const YAML::Node& config);
std::vector<Obstacle> extract_obs(const YAML::Node& config);
    
#endif // map_H
