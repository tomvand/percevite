#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <cmath>

#include "percevite_pprzlink.hpp"
#include "percevite_messages.h"

// TODO http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
// TODO http://developer.parrot.com/docs/slamdunk/#code

// Goals:
// - Check depth map for:
//		- Distance to obstacle
//			- In what direction? Obtain vector from /pose or paparazzi?
//			- What if vector is out of view? -> Set 'safe distance' output to zero.
//		- Depth information available
//
// Planned program outline
// Initialization:
// 1. Initialize ROS
//		a. Subscribe to
//			- /depth_map/image
//			- /depth_map/camera_info -> for focal length
//		b. Advertise:
//			- /percevite/debug_image?
// 2. Use IMU to measure pitch relative to Bebop frame?
// 3. Output 'starting' status to paparazzi?
//
// Run:
// - Get movement direction. Options:
//		- From autopilot mission (e.g. waypoint in body frame)
//		- From autopilot INS
//		- From slamdunk odometry
//		- Or both plan and actual movement?
//
//		- If out of view: send yaw command to autopilot to center movement direction in image.
// - Get depth image
//		- Determine ROI from movement direction
//		- For pixels in depth image:
//			- Number of ok pixels (finite and inside image)

// Maybe: min-box-filter image, send highest-distance bearing to autopilot for alternative course?

namespace {

void imageCallback(
		const sensor_msgs::ImageConstPtr &color,
		const sensor_msgs::ImageConstPtr &depth) {
	cv_bridge::CvImageConstPtr cv_color_ptr = cv_bridge::toCvShare(color, "bgr8");
	cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth);

	// TODO Find relevant ROI (now takes entire region)
	cv::Rect roi(0, cv_depth_ptr->image.rows / 4,
			cv_depth_ptr->image.cols, cv_depth_ptr->image.rows / 2);
	cv::Mat depth_roi;
	cv_depth_ptr->image(roi).copyTo(depth_roi);

	// Calulate ROI stats
	int valid_pixels = 0;
	int distance_histogram[150];
	memset(distance_histogram, 0, sizeof(distance_histogram));
	for (int c = 0; c < depth_roi.cols; ++c) {
		for (int r = 0; r < depth_roi.rows; ++r) {
			if (std::isfinite(depth_roi.at<float>(r, c))) {
				++valid_pixels;
				int dist = depth_roi.at<float>(r, c) * 10.0;
				dist = dist < 0 ? 0 : dist;
				dist = dist > 149 ? 149 : dist;
				++(distance_histogram[dist]);
			}
		}
	}
	ROS_INFO("Valid pixels: %.0f %%",
			100.0 * valid_pixels / (depth_roi.cols * depth_roi.rows));
	int percentile = 10;
	int percentile_count = valid_pixels * percentile / 100;
	int dist;
	for(dist = 0; percentile_count > 0 && dist < 150; ++dist) {
		percentile_count -= distance_histogram[dist];
	}
	ROS_INFO("%d-percentile distance: %.1fm", percentile, dist / 10.0);
	// NOTE: For histogram, the discrete and limited disparities might be a better choice!


	// ************************************************************************
	// For testing: highlight pixels smaller than dist in color image
	// ************************************************************************
	cv::Mat debug;
	cv_color_ptr->image.copyTo(debug);
	int ratio = debug.cols / cv_depth_ptr->image.cols;
	for(int x = 0; x < debug.cols; ++x) {
		for(int y = 0; y < debug.rows; ++y) {
			int x_roi = x / ratio;
			int y_roi = (y - debug.rows / 4) / ratio;
			bool inside_roi = (x_roi >= 0) && (x_roi < depth_roi.cols) &&
					(y_roi >= 0) && (y_roi < depth_roi.rows);
			if(inside_roi && depth_roi.at<float>(y_roi, x_roi) * 10.0 < dist) {
				cv::Vec3b color = debug.at<cv::Vec3b>(y, x);
				color.val[2] = 255;
				debug.at<cv::Vec3b>(y, x) = color;
			} else if (!inside_roi || !std::isfinite(depth_roi.at<float>(y_roi, x_roi))) {
				cv::Vec3b color = debug.at<cv::Vec3b>(y, x);
				color.val[0] = 255;
				debug.at<cv::Vec3b>(y, x) = color;
			}
		}
	}

	// OLD test code
	cv::imshow("Depth", depth_roi / 10.0);
	cv::imshow("Debug", debug);

	char key = cv::waitKey(1);
	if (key == 27 || key == 'q') {
		ros::shutdown();
	}

	//**************************************************************************
	// For testing: write dummy text to pprzlink
	//**************************************************************************
	SlamdunkToPaparazziMsg msg;
	msg.safe_distance = (uint8_t)dist;
	pprzlink.write(sizeof(msg), &msg.bytes);

	//**************************************************************************
	// For testing: read dummy text from pprzlink
	//**************************************************************************
	PaparazziToSlamdunkMsg msg2;
	if(pprzlink.read(sizeof(msg2), &msg2.bytes)) {
		ROS_INFO("Received message: [%s]\n", msg2.text);
	}

}

} // namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "percevite");
	ros::NodeHandle nh;

	// Subscribe to two image topics
	image_transport::ImageTransport it(nh);
	image_transport::SubscriberFilter sub_depth(it, "/depth_map/image", 3);
	image_transport::SubscriberFilter sub_color(it,
			"/left_rgb_rect/image_rect_color", 10);

	// Synchronize the two topics
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
			sub_color, sub_depth, 10);
	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

	// Initialize pprzlink
	pprzlink.init();

	ros::spin();

	return 0;
}
