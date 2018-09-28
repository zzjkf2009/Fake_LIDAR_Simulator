/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-27T19:22:56-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: fake_scanner.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-27T19:23:12-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 * @brief: This is the source file for node fake_scanner that will publish the sensor_msgs/LaserScan
 * the parameters
 */



#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
        ros::init(argc, argv, "laser_scan_publisher");
        ros::NodeHandle n;
        if(argc != 8) {
                ROS_ERROR("Invalid number of arguments for: num_readings, distance, angle_min, angle_max,range_min,range_max, topicName");
                return 1;
        }
        ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>(argv[7], 500);
        unsigned int num_readings = atof(argv[1]);
        double laser_frequency = 60;
        double ranges[num_readings];
        int distance = atof(argv[2]);
        ros::Rate r(5.0);

        while(n.ok()) {
                //generate some fake data for our laser scan
                ros::Time scan_time = ros::Time::now();
                sensor_msgs::LaserScan scan;
                scan.header.stamp = scan_time;
                scan.header.frame_id = "laser_frame";
                scan.angle_min = atof(argv[3]);
                scan.angle_max = atof(argv[4]);
                scan.angle_increment = (atof(argv[4]) - atof(argv[3])) / num_readings;
                scan.time_increment = (1 / laser_frequency) / (num_readings);
                scan.range_min = atof(argv[5]);
                scan.range_max = atof(argv[6]);
                scan.ranges.resize(num_readings);
                scan.intensities.resize(num_readings);
                for(unsigned int i = 0; i < num_readings; ++i) {
                        scan.ranges[i] = distance;
                }
                scan_pub.publish(scan);
                r.sleep();
        }
}
