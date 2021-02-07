/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-28T09:55:28-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: fake_scanner_external.h
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-28T09:55:46-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */
 #ifndef FAKE_SCANNER_EXTERNAL_H
 #define FAKE_SCANNER_EXTERNAL_H

 #include <ros/ros.h>
 #include <sensor_msgs/LaserScan.h>

class Scan2 {
public:
Scan2();
private:
ros::NodeHandle n;
ros::Publisher scan_pub;
ros::Subscriber scan_sub;
void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);
};

#endif
