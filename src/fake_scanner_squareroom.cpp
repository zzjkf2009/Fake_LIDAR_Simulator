#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class Scan2 {
public:
Scan2();
private:
ros::NodeHandle n;
ros::Publisher scan_pub;
ros::Subscriber scan_sub;
tf::TransformBroadcaster br;
tf::TransformListener listener;
tf::StampedTransform transform;
double x_p = 0.0;
double y_p = 0.0;
double yaw_p = 0.0;
double dx;
double dy;
double dyaw;
bool getx = ros::param::get("~x_incremental", dx);
bool gety = ros::param::get("~y_incremental", dy);
bool getyaw = ros::param::get("~yaw_incremental", dyaw);
void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);
void dynamic_broadcaster(double dx, double dy,double dyaw);
};

Scan2::Scan2()
{
        scan_pub = n.advertise<sensor_msgs::LaserScan>("/fake_scan",500);
        scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1,
                                                       &Scan2::scanCallBack, this);
        if(!getx || !gety || !getyaw) {
                ROS_FATAL_STREAM("Can't get parameters");
        }
        while(n.ok()) {
                dynamic_broadcaster(dx,dy,dyaw);
                ros::Duration(0.5).sleep();
                ros::spinOnce();
        }

}
void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr&
                         scan2)
{
        int ranges = scan2->ranges.size();
//populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan2->header.stamp;
        scan.header.frame_id = "laser_frame";
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


void Scan2::dynamic_broadcaster(double dx, double dy,double dyaw){
        transform.setOrigin( tf::Vector3(x_p, y_p, yaw_p) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
        x_p += dx;
        y_p += dy;
        yaw_p += dyaw;
}

int main(int argc, char** argv){
        ros::init(argc, argv, "fake_scanner");
        Scan2 scan2;
}
