# Multi Robot Simulator
The package is used to simulate the unicycle robots in 2D enviroment. Robots are shown in an interective enviroment where they can be moved easily.Every robot has a Lidar sensor.
The output of the lidar also can be seen
![Screenshot from 2024-03-26 20-15-32](https://github.com/tulsikumar-1/rp_project/assets/104934568/e622f9fc-1288-40b5-818e-b1a2b28784ab)


## How to run
Download to repo and paste it in your catkin workspace

Source the setup file using

```sh
source devel/setup.bash
```

Build the package in the catkin workspace using following


```sh
catkin build
```

After build process is completed again source the setup.bash

```sh
source devel/setup.bash
```
The configration of the simulator can be editted in the config.yaml file which can be located inside the config folder in the package


Config file takes robots description and map discription

Map path can be changed according to current maps and also resolution is to be changed according to map.yaml file.

Maps can be located inside the maps folder in the package.

Once config.yaml file is ready then following command can be used to run the package


## Only simulator

```sh
rosrun xy_simulator main
```
## Simulator with naivation stack

```sh
roslaunch xy_simulator navigation navigation.launch map_file:=map.yaml
```

map.yaml needs to be changed according to map being used for example in case of "cappero_map.yaml" following cmd is used

```sh
roslaunch xy_simulator navigation navigation.launch map_file:=cappero_map.yaml
```



