#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Simple ROS node to convert disparity images to depth images

image_transport::Publisher depth_pub;
image_transport::Publisher disp_pub;

float f = 425.0;
float B = 0.20;

int output_size = 96;
cv::Mat disp_s;
cv::Mat depth;

void disparityImageCallback(const stereo_msgs::DisparityImage& msg) {
  cv::Mat_<float> disp(msg.image.height, msg.image.width, (float*)(&(msg.image.data[0])));
//  cv::imshow("disp", disp);
//  cv::waitKey(1);
  cv::Mat invalid_mask(disp < 0.0);
  disp.setTo(std::numeric_limits<float>::quiet_NaN(), invalid_mask);
//  cv::imshow("invalids", invalid_mask);
//  cv::waitKey(1);

  // Downscale disparity map
  cv::resize(disp, disp_s, cv::Size(output_size, output_size), 0, 0, cv::INTER_NEAREST);
  // Convert into depth using f and B
  depth = B * f / disp_s;
  depth.setTo(1000.0, (disp_s == 0.0) | (depth > 1000.0)); // To-do: check 0 disparity behavior on SLAMDunk
  // Publish
  sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(msg.header, sensor_msgs::image_encodings::TYPE_32FC1, depth).toImageMsg();
  depth_pub.publish(depth_msg);
  sensor_msgs::ImagePtr disp_msg = cv_bridge::CvImage(msg.header, sensor_msgs::image_encodings::TYPE_32FC1, disp_s).toImageMsg();
  disp_pub.publish(disp_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "disp2depth");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("disp_image", 1, disparityImageCallback);
  image_transport::ImageTransport it(nh);
  disp_pub = it.advertise("disp_out", 1);
  depth_pub = it.advertise("depth_out", 1);

  nh.getParam("output_size", output_size);
  ROS_INFO_STREAM("Output size: " << output_size);
  nh.getParam("f", f);
  ROS_INFO_STREAM("f: " << f << "px");
  nh.getParam("B", B);
  ROS_INFO_STREAM("B: " << B << "m");

  ros::spin();
}
