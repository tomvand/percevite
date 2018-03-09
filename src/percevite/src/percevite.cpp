#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

// TODO http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
// TODO http://developer.parrot.com/docs/slamdunk/#code

void imageCallback(
		const sensor_msgs::ImageConstPtr &color,
		const sensor_msgs::ImageConstPtr &depth) {
	cv_bridge::CvImageConstPtr cv_color_ptr = cv_bridge::toCvShare(color, "bgr8");
	cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth);

	cv::Mat cv_depth_8uc1;
	cv_depth_ptr->image.convertTo(cv_depth_8uc1, CV_8UC1, -255.0 / 10.0, 255.0);

	cv::imshow("Depth", cv_depth_8uc1);
	cv::imshow("Color", cv_color_ptr->image);

	char key = cv::waitKey(1);
	if (key == 27 || key == 'q') {
		ros::shutdown();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "percevite");
	ros::NodeHandle n;

	// Subscribe to two image topics
	image_transport::ImageTransport it(n);
	image_transport::SubscriberFilter sub_depth(it, "/depth_map/image", 3);
	image_transport::SubscriberFilter sub_color(it, "/left_rgb_rect/image_rect_color", 10);

	// Synchronize the two topics
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
			sub_color, sub_depth, 10);
	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

	ros::spin();

	return 0;
}
