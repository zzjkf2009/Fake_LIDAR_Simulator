/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-28T09:53:43-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: fake_scanner.h
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-28T09:53:49-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */
#ifndef FAKE_SCANNER_H
#define FAKE_SCANNER_H


 #include <ros/ros.h>
 #include <sensor_msgs/LaserScan.h>
 #include <tf/transform_broadcaster.h>

class Scan {
public:
Scan(double num_readings, double distance, double angle_min, double angle_max, double range_min, double range_max, std::string topicName);
double getNum() const;
private:
ros::NodeHandle n;
ros::Publisher scan_pub;
std::string topicName_;
double num_readings_;
double angle_min_;
double angle_max_;
double range_min_;
double range_max_;
double distance_;
};

#endif
