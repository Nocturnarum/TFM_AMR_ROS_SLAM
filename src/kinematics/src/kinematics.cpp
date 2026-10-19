#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <cmath>

#define diameter 0.065
#define radius 0.0325
#define separation 0.38
#define max_rpm 61.0 
#define pi 3.1416
#define cmd_timeout 0.5


ros::Publisher vel_motor_pub;
ros::Time last_cmd_time;

double linear = 0.0;
double angular = 0.0;
double vel_L = 0.0;
double vel_R = 0.0;
double rpm_L = 0.0;
double rpm_R = 0.0;
bool   msg_in = false;

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg){
	linear = msg->linear.x;
	angular = msg->angular.z;
	last_cmd_time = ros::Time::now();
}

int main(int argc, char **argv)
{
    	ros::init(argc, argv, "kinematics_node");
    	ros::NodeHandle nh;
	
	ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_Callback);
	vel_motor_pub = nh.advertise<geometry_msgs::Twist>("/vel_motor", 10);

	ros::Rate loop_rate(10);
	while (ros::ok()){
		ros::spinOnce();
		
		const bool reset = last_cmd_time.isZero() || (ros::Time::now() - last_cmd_time).toSec() > cmd_timeout;	
		
		if(reset){
			rpm_L = 0.0;
			rpm_R = 0.0;
		}
		else{
			vel_L = linear - (angular * separation / 2);
			vel_R = linear + (angular * separation / 2);
			rpm_L = vel_L * 60 / (2 * pi * radius);
			rpm_R = vel_R * 60 / (2 * pi * radius);
			msg_in = false;
		}		
		
		geometry_msgs::Twist twist;
		twist.linear.x = rpm_L;
		twist.linear.y = rpm_R;
		vel_motor_pub.publish(twist);

		loop_rate.sleep();
	}

    	return 0;
}
