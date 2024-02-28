#ifndef map_H
#define map_H


#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <iostream>
#include "robots.h"



  
struct MapConfig {
    std::string path;
    double width;
    double height;
    double resolution;
 
} ; 

MapConfig extract_map(const YAML::Node& config);
    
#endif // map_H
