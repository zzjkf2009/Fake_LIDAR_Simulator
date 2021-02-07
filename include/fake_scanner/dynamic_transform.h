/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-28T09:44:48-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: dynamic_transform.h
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-28T09:51:47-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */
#ifndef DYNAMIC_TRANSFORM_H
#define DYNAMIC_TRANSFORM_H

 #include <ros/ros.h>
 #include <tf/transform_broadcaster.h>

class Dynamic_Transform {
public:
Dynamic_Transform(double init_x,double init_y,double init_z,double init_yaw,double x_incremental,double y_incremental,double yaw_incremental);
double getY_incremental() const;
private:
ros::NodeHandle n;
tf::TransformBroadcaster br;
tf::Transform transform;
double x_p;
double y_p;
double z_p;
double yaw_p;
double dx;
double dy;
double dyaw;
};

#endif
