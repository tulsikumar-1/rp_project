#ifndef ROBOTS_H
#define ROBOTS_H

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <iostream>

struct Pose {
    double x;
    double y;
    double theta;
};

struct Dimensions {
    double radius;
};

struct Color {
    double r;
    double g;
    double b;
    double a;
};

struct Lidar {
    int num_beams;
    double min_range;
    double max_range;
};

struct Robot {
    std::string name;
    std::string frame_id;
    Pose pose;
    Dimensions dimensions;
    Color color;
    Lidar lidar;
    double max_vel;
};

std::vector<Robot> extractRobots(const YAML::Node& config);

#endif // ROBOTS_H

