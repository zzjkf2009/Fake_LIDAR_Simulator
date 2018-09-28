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

class Dynamic_Transform {
public:
Dynamic_Transform(double init_x,double init_y,double init_z,double init_yaw,double x_incremental,double y_incremental,double yaw_incremental);
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
/**
 * [Dynamic_Transform::Dynamic_Transform constructor Dynamically broadcast frame from robot to world by
 * adding the x_incremental, y_incremental, yaw_incremental values]

 */
Dynamic_Transform::Dynamic_Transform(double init_x,double init_y,double init_z,double init_yaw,double x_incremental,double y_incremental,double yaw_incremental) :
        x_p(init_x),y_p(init_y),z_p(init_z),yaw_p(init_yaw),dx(x_incremental),dy(y_incremental),dyaw(yaw_incremental){

        while (n.ok()) {
                transform.setOrigin( tf::Vector3(x_p, y_p, z_p) );
                tf::Quaternion q;
                q.setRPY(0, 0, yaw_p);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
                x_p += dx;
                y_p += dy;
                yaw_p += dyaw;
                ros::spinOnce();
                ros::Duration(0.1).sleep();
        }
}

int main(int argc, char** argv){
        ros::init(argc, argv, "dynamic_tf_broadcaster");
        // Read the initial transformation via arguments, if no arguments provide return 1
        if(argc != 5) {
                ROS_ERROR("Invalid number of arguments for: init_x, init_y, init_z, init_yaw");
                return 1;
        }

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
        Dynamic_Transform dynamic_transform(x_p,y_p,z_p,yaw_p,dx,dy,dyaw);
        return 0;
};
