# LIDAR Simulator


## Overview
This is the package that simulates the LIDAR data by publishing [laserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html) message. The robot is "moving" by creating a dynamic robot frame relative to world. Three nodes are created, one creates a dynamic transformation, one publishes made-up LaserScan message, one subscribes laserScan message from other resources and publish new LaserScan message.  By setting the BagMode to be true of false, it will choose to use the made-up LaserScan data or LaserScan data from bag file. More details about the parameters and arguments can be found in the launch file. Bag files can be created by recording the LaserScan data from the LIDAR on [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) in Gazebo or from the real scanning data using [RPLIDAR A1](wiki.ros.org/rplidar).


## Build and Run
- Run the simulation via launch file
```
roslaunch fake_scanner lidar_simulator.launch
roslaunch fake_scanner lidar_simulator.launch bagMode:=true laser_offset_x:=0.2 laser_offset_y:=0.1 laser_offset_z:=0.1 x_incremental:=0.01 y_incremental:=0.01 yaw_incremental:=0.01 init_transform:="0.1 0.1 0.2 0.1" LIDAR_parameters:="100 2.0 -3.14 3.14 0 6.2"
```
- Create ROS bag which contains /scan info

```
roslaunch fake_scanner turtlebot3_scan.launch
```
- To get more info about the package
```
rqt_graph
rosparam list
rostopic list
```

Unit test using gtest
```
catkin_make run_tests_fake_scanner_gtest
```
- ![gtest](https://github.com/zzjkf2009/Fake_LIDAR_Simulator/blob/devel/image/Unit_test.png)
## Structure
The graph representation:
- ![rqt](https://github.com/zzjkf2009/Fake_LIDAR_Simulator/blob/devel/image/rqt_graph.png)
- ![summary](https://github.com/zzjkf2009/Fake_LIDAR_Simulator/blob/devel/image/Summary.png)

##### Node: dynamic_transform
This is the node: dynamic_transform, which will define the relationship between frame */robot* to the */world*. The initial transformation (translation [x, y, z], rotation [0, 0, yaw]) was defined via arguments, and the x_incremental, y_incremental, yaw_incremental are ros parameters which can be set and got via ros parameter server. The initial value can be defined in launch file.
- ros parameters : x_incremental, y_incremental,yaw_incremental
- input arguments : init_x, init_y, init_z, init_yaw

##### Node: fake_scanner
This is the node: fake_scanner, which publishes the made-up LaserScan data (circle). The parameters of the data can be set via arguments.
- input arguments : num_readings, distance, angle_min, angle_max, range_min, range_max, topicName

##### Node: fake_scanner_external
This is the node: fake_scanner_external, which will subscribe the */scan* topic from other sources (eg. running nodes, bag files) and the publish a new */fake_scan* topic (*sensor_msgs/LaserScan*), which basically inherit all the parameters (eg. min/max range, angle_increment, min/max angle etc) from the */scan*.
##### Node: static_transform_publisher
This is node: static_transform_publisher, which will create a static transformation between */robot* and *laser_frame*. The translation and rotation arguments can be defined via the launch file.
##### Node: bagplay
This is node: bagplay, which will read the contents of one or more bag file, and plays them back in a time-synchronized fashion.  Pre-recorded bag files are located in bag folder. The path of bag file can be set via launch argument.
##### Node: rviz
Rviz is executed to display the LaserScan data and tf frames.

## Video Demo
Create the made-up LaserScan data as a circle and robot is moving straight
[![Refer video](https://img.youtube.com/vi/JdwR-_Dt6RA/0.jpg)](https://youtu.be/JdwR-_Dt6RA) \
Subscribe the LaserScan data from bag file (*/scan*) and publish new scan topic
 [![Refer video](https://img.youtube.com/vi/tNV9SA3vey8/0.jpg)](https://youtu.be/tNV9SA3vey8)

 ## Hardware Device
There are a few low-price LIDAR devices, such as [Slamtech series of LIDARs](https://www.dfrobot.com/product-1125.html), RPLIDAR A1 ($99) has a 8m range and 40000 points per second. Based on this [blog](https://diyrobocars.com/2018/08/04/lidar-slam-without-ros-on-cheap-computing-boards/), we can create a good performance (200 points/sec) laser scanner with a reasonable price (under $200) by using RPLIDAR and odroid XU4. The detailed descriptions and data sheet can be find at [here](http://bucket.download.slamtec.com/6fad02c42af6da33f89fbc043c5f165e2b222e0d/rplidar_interface_protocol_en.pdf). The LIDAR used in Turtlebot3 is [LDS-01](http://www.robotis.us/360-laser-distance-sensor-lds-01-lidar/), which is cost more ($179) than the RPLIDAR.
