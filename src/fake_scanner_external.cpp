/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-27T16:16:21-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: fake_scanner_external.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-27T16:53:51-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 * @ brif: This is the source file for the node: fake_scanner_external.It will subscribe
 * the /scan topic from other sources (eg. running nodes, bag files) and the publish a new /fake_scan topic
 * (sensor_msgs/LaserScan),which basically inherit all the parameters (eg. min/max range, angle_increment,min/max angle etc)
 * from the /scan.
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "fake_scanner_external.h"

/**
 * [Scan2::Scan2 constructor for the Scan2 class]
 */
Scan2::Scan2()
{       // Create a subscriber and a pubulisher
        scan_pub = n.advertise<sensor_msgs::LaserScan>("/fake_scan",500);
        scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1,
                                                       &Scan2::scanCallBack, this);
}

/**
 * [Scan2::scanCallBack callbaclk function that will read and publish the LaserScan messgae]
 * @param scan2 [const sensor_msgs::LaserScan::ConstPtr]
 */
void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr&
                         scan2)
{
        int ranges = scan2->ranges.size();
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan2->header.stamp;
        scan.header.frame_id = "laser_frame"; // Name the frame ID as laser_frame
        scan.angle_min = scan2->angle_min;
        scan.angle_max = scan2->angle_max;
        scan.angle_increment = scan2->angle_increment;
        scan.time_increment = scan2->time_increment;
        scan.range_min = 0.0;
        scan.range_max = 6.0;
        scan.ranges.resize(ranges);
        for(int i = 0; i < ranges; ++i) {
                scan.ranges[i] = scan2->ranges[i] + 1;
        }
        scan_pub.publish(scan);
}

int main(int argc, char** argv){
        ros::init(argc, argv, "fake_scanner_external");
        Scan2 scan2;
        ros::spin();
        ros::Duration(0.5).sleep();
}
