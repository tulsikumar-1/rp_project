# Multi Robot Simulator
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

```sh
roslaunch xy_simulator navigation navigation.launch map_file:="map.yaml" robot_name:=Bot1
```


