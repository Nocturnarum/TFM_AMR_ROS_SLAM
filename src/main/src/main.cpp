#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#define timeout 0.5

ros::Publisher vel_pub;

double linear_joy = 0.0;
double angular_joy = 0.0;
double linear_slam = 0.0;
double angular_slam = 0.0;
ros::Time last_joy_time;
ros::Time last_slam_time;
bool   mode_manual = true;

void modeCallback(const std_msgs::Bool::ConstPtr& msg){
	mode_manual = msg->data;
	ROS_INFO("Modo: %s", mode_manual ? "MANUAL" : "AUTO");
}	
void cmd_vel_joy_Callback(const geometry_msgs::Twist::ConstPtr& msg){
	linear_joy = msg->linear.x;
	angular_joy = msg->angular.z;
	last_joy_time = ros::Time::now();
}
void cmd_vel_slam_Callback(const geometry_msgs::Twist::ConstPtr& msg){
	linear_slam = msg->linear.x;
	angular_slam = msg->angular.z;
	last_slam_time = ros::Time::now();
}
void timer_Callback(const ros::TimerEvent&){
	geometry_msgs::Twist twist;

	if(mode_manual){
		//const bool ok = !last_joy_time.isZero() && (ros::Time::now() - last_joy_time).toSec() <= timeout;		
		//twist.linear.x = ok ? linear_joy : 0.0;
		//twist.angular.z = ok ? angular_joy : 0.0;		
		twist.linear.x = linear_joy;
		twist.angular.z = angular_joy;
	}
	else{
		const bool ok = !last_slam_time.isZero() && (ros::Time::now() - last_slam_time).toSec() <= timeout;
		twist.linear.x = ok ? linear_slam : 0.0;
		twist.angular.z = ok ? angular_slam : 0.0;
		//twist.linear.x = linear_slam;
		//twist.angular.z = angular_slam;
	}

	vel_pub.publish(twist);

}

int main(int argc, char **argv)
{
    	ros::init(argc, argv, "main_node");
    	ros::NodeHandle nh;
	
	ros::Subscriber mode_manual_sub = nh.subscribe("/mode_manual", 10, modeCallback);
	ros::Subscriber cmd_vel_joy_sub = nh.subscribe("/cmd_vel_joy", 10, cmd_vel_joy_Callback);
	ros::Subscriber cmd_vel_slam_sub = nh.subscribe("/cmd_vel_slam", 10, cmd_vel_slam_Callback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_Callback);	
	
	ROS_INFO("Nodo principal iniciado (main_node)");
	
	ros::spin();

    	return 0;
}
