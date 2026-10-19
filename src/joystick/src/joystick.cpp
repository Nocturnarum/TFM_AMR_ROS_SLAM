#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>          
#include <visualization_msgs/Marker.h>  

#define max_linear 0.180 //0.208m/s
#define max_angular 1.00 //1.19rad/s

ros::Publisher vel_pub;
ros::Publisher mode_pub;
ros::Publisher home_pub;
ros::Publisher marker_pub;   
std::string perfil = "";     

double scale_linear = max_linear;
double scale_angular = max_angular;
double linear = 0.0;
double angular = 0.0;
bool   mode_manual = true;
bool   last_b12 = 0;
bool   last_b13 = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	linear = joy->axes[1];
	angular = joy->axes[2];
	if (joy->buttons[12] == true && last_b12 == 0){
		mode_manual = !mode_manual;
		std_msgs::Bool mode_manual_msg;
		mode_manual_msg.data = mode_manual;
		mode_pub.publish(mode_manual_msg);
	}
	last_b12 = joy->buttons[12];

	if (joy->buttons[13] == true && last_b13 == 0 && mode_manual == false){
		std_msgs::Empty msg;
		home_pub.publish(msg);
	}
	last_b13 = joy->buttons[13];

	geometry_msgs::Twist twist;
	twist.linear.x = scale_linear * linear;
	twist.angular.z = scale_angular * angular;
	vel_pub.publish(twist);
}

void perfilCallback(const std_msgs::String::ConstPtr& msg){
	perfil = msg->data;
}

void timer_Callback(const ros::TimerEvent&){
	visualization_msgs::Marker m;
	m.header.frame_id = "base_link";
	m.header.stamp = ros::Time::now();
	m.ns = "modo";
	m.id = 0;
	m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m.action = visualization_msgs::Marker::ADD;
	m.pose.position.z = 0.4;
	m.pose.orientation.w = 1.0;
	m.scale.z = 0.12;
	m.color.a = 1.0;
	if (mode_manual){                       
		m.text = "MANUAL";
		m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0;
	} else if (perfil == "estrecho"){       
		m.text = "AUTO-ESTRECHO";
		m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0;
	} else if (perfil == "amplio"){         
		m.text = "AUTO-AMPLIO";
		m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0;
	} else {                                
		m.text = "AUTO";
		m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0;
	}
	marker_pub.publish(m);
}	

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/joy", 10, joyCallback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_joy", 1, true);
	mode_pub = nh.advertise<std_msgs::Bool>("/mode_manual", 1, true);
	home_pub = nh.advertise<std_msgs::Empty>("/home", 1, true);
	marker_pub = nh.advertise<visualization_msgs::Marker>("/indicador_modo", 1);       
	ros::Subscriber sub_perfil = nh.subscribe("/perfil_navegacion", 1, perfilCallback); 

	std_msgs::Bool mode_manual_msg;
	mode_manual_msg.data = mode_manual;
	mode_pub.publish(mode_manual_msg);
	
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_Callback);	

	ros::spin();

	return 0;
}
