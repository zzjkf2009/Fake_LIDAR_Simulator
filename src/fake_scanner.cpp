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
#include "fake_scanner.h"

/**
 * [Scan::Scan constructor Publish made-up LaserScan message]
 */
Scan::Scan(double num_readings, double distance, double angle_min, double angle_max, double range_min, double range_max, std::string topicName) :
        num_readings_(num_readings),distance_(distance),angle_min_(angle_min),angle_max_(angle_max),range_min_(range_min),
        range_max_(range_max),topicName_(topicName){
        while(n.ok()) {
                sensor_msgs::LaserScan scan;
                scan_pub = n.advertise<sensor_msgs::LaserScan>(topicName_,500);
                scan.header.stamp = ros::Time::now();
                scan.header.frame_id = "laser_frame";
                scan.angle_min = angle_min_;
                scan.angle_max = angle_max_;
                scan.angle_increment = (angle_max_ - angle_min_) / num_readings_;
                scan.time_increment = (1 / 60) / (num_readings_);
                scan.range_min = range_min_;
                scan.range_max = range_max_;
                scan.ranges.resize(num_readings_);
                for(unsigned int i = 0; i < num_readings_; ++i) {
                        scan.ranges[i] = distance_;
                }
                scan_pub.publish(scan);
                ros::spinOnce();
                ros::Duration(0.5).sleep();
        }
}
double Scan::getNum() const {
        return num_readings_;
}
int main(int argc, char** argv){
        ros::init(argc, argv, "laser_scan_publisher");
        if(argc != 8) {
                ROS_ERROR("Invalid number of arguments for: num_readings, distance, angle_min, angle_max,range_min,range_max, topicName");
                return 1;
        }
        std::string topicname(argv[7]);
        Scan(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),topicname);
}
