#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>

void setDebugLogger(void) {
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	ROS_INFO("Pose update: [%f]", msg->pose.position.x);
}

int main(int argc, char **argv) {
	ROS_INFO("ros::init()");
	ros::init(argc, argv, "percevite");
	ROS_INFO("ros::NodeHandle");
	ros::NodeHandle n;
	ROS_INFO("ros::Subscriber");
	ros::Subscriber sub = n.subscribe("/pose", 1, &poseCallback);

	ROS_INFO("ros::spin()");
	ros::spin();

	return 0;
}
