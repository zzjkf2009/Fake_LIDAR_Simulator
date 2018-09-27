# LIDAR Simulator


## LIDAR

The simulated LIDAR is the [Velodyne VLP-16](https://velodynelidar.com/vlp-16.html), which has a 100 m range, up to 600,000 points per second, 360 degree.  


##
roslaunch fake_scanner lidar_simulator.launch

rosbag record tf scan

roslaunch fake_scanner turtlebot3_scan.launch

rosparam get  
rosparam set
