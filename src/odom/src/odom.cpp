#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#define diameter 0.065
#define radius 0.0325
#define separation 0.38
#define max_rpm 61.0 
#define pi 3.1416 // o M_PI
#define cmd_timeout 0.5


ros::Publisher odom_pub;
ros::Time last_msg_time;

double linear = 0.0;
double angular = 0.0;
double vel_L = 0.0;
double vel_R = 0.0;
double rpm_L = 0.0;
double rpm_R = 0.0;
bool   msg_in = false;

double x = 0.0;
double y = 0.0;
double th = 0.0;

void data_motor_Callback(const geometry_msgs::Twist::ConstPtr& msg){
	rpm_L = msg->linear.x;
	rpm_R = -msg->angular.x;
	msg_in = true;
	last_msg_time = ros::Time::now();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_node");
	ros::NodeHandle nh;
	
	ros::Subscriber data_motor_sub = nh.subscribe("/data_motor", 1, data_motor_Callback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
	tf::TransformBroadcaster tf_br;

	std::string odom_frame = "odom"; 
	std::string base_frame = "base_link";
	
	ros::Time last_time = ros::Time::now();

	ros::Rate loop_rate(20);
	while (ros::ok()){
		ros::spinOnce();
		ros::Time now = ros::Time::now();
		
		double dt = (now-last_time).toSec();
		last_time = now;

		if(dt <= 0.0){
			loop_rate.sleep();
			continue;
		}
		if((now - last_msg_time).toSec() >= cmd_timeout){
			rpm_L = 0.0;
			rpm_R = 0.0;
		}

		vel_L = rpm_L * 2.0 * M_PI * radius / 60.0;
		vel_R = rpm_R * 2.0 * M_PI * radius / 60.0;

		linear = (vel_R + vel_L) / 2.0;
		angular = (vel_R - vel_L) / separation;

		double d_th = angular * dt;

		x += linear * cos(th + d_th / 2.0) * dt;
		y += linear * sin(th + d_th / 2.0) * dt;
		th += d_th;
		th = atan2(sin(th), cos(th));
		
		geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(th);
		geometry_msgs::TransformStamped t;
		t.header.stamp = now;
		t.header.frame_id = odom_frame;
		t.child_frame_id = base_frame;
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = 0.0;
		t.transform.rotation = q;
		tf_br.sendTransform(t);

		nav_msgs::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = odom_frame;
		odom.child_frame_id = base_frame;
		odom.pose.pose.position.x = x; 
		odom.pose.pose.position.y = y; 
		odom.pose.pose.orientation = q;
		odom.twist.twist.linear.x = linear;
		odom.twist.twist.angular.z = angular; 

		odom.pose.covariance[0] = 0.01;
		odom.pose.covariance[7] = 0.01;
		odom.pose.covariance[14] = 1e6;
		odom.pose.covariance[21] = 1e6;
		odom.pose.covariance[28] = 1e6;
		odom.pose.covariance[35] = 0.03;
		odom.twist.covariance[0] = 0.01;
		odom.twist.covariance[35] = 0.03;

		odom_pub.publish(odom);

		loop_rate.sleep();
	}

    return 0;
}
