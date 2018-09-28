/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-25T17:04:35-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: dynamic_transform.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-25T17:04:41-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 * @brif: this is the source file for the node: dynamic_transform, which will
 * define the relationship between frame /world and /robot. The initial transformation
 * (x, y, yaw) was defined via arguments, and the x_incremental, y_incremental,
 * yaw_incremental are ros parameters which can be set and got via ros parameter
 * server. The initial value is defined in launch file or all set to zero
 */



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
        ros::init(argc, argv, "dynamic_tf_broadcaster");
        // Read the initial transformation via arguments, if no arguments provide return 1
        if(argc != 5) {
                ROS_ERROR("Invalid number of arguments for: init_x, init_y, init_z, init_yaw");
                return 1;
        }

        ros::NodeHandle node;
        tf::TransformBroadcaster br;
        tf::Transform transform;
        ros::Rate rate(10.0);
        // Read the initial translation [x_p, y_p,z_p] and roation [0,0,yaw] from argument
        double x_p = atof(argv[1]);
        double y_p = atof(argv[2]);
        double z_p = atof(argv[3]);
        double yaw_p = atof(argv[4]);
        // Set incremental value for x, y and yaw via parameters, those parameters are pre-defined in launch file
        double dx;
        double dy;
        double dyaw;
        bool getdx = ros::param::get("~x_incremental", dx);
        bool getdy = ros::param::get("~y_incremental", dy);
        bool getdyaw = ros::param::get("~yaw_incremental", dyaw);

        // If any of those arguments are not defined, set dx,dy,dyaw to zero
        if(!getdx || !getdy || !getdyaw) {
                ROS_INFO_STREAM("Can't get parameters");
                dx = 0.0; dy = 0.0; dyaw = 0.0;
        }
        // Dynamically broadcast the transformation between /world frame and /robot frame by incremnt x, y and yaw for a certain value in a defined rate
        while (node.ok()) {
                transform.setOrigin( tf::Vector3(x_p, y_p, z_p) );
                tf::Quaternion q;
                q.setRPY(0, 0, yaw_p);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
                x_p += dx;
                y_p += dy;
                yaw_p += dyaw;
                rate.sleep();
        }
        return 0;
};
