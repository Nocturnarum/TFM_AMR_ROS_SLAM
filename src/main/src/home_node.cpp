#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher goal_pub;

void homeCallback(const std_msgs::Empty::ConstPtr&)
{
	geometry_msgs::PoseStamped home;
	home.header.frame_id = "map";
	home.header.stamp = ros::Time::now();
	home.pose.position.x = 0.0;
	home.pose.position.y = 0.0;
	home.pose.orientation.w = 1.0;   // theta = 0
	goal_pub.publish(home);
	ROS_INFO("Volviendo a home (0, 0, 0)");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "home_node");
	ros::NodeHandle nh;

	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	ros::Subscriber sub = nh.subscribe("/home", 1, homeCallback);

	ros::spin();

	return 0;
}
