#ifndef DETECT_TRAFFIC_SIGN_2_H
#define DETECT_TRAFFIC_SIGN_2_H

#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

namespace sign_code{
  enum{YELLOW, GREEN, RED, NONE};
}

typedef struct{
  double min_dist;
  double param1;
  double param2;
  double min_radius;
  double max_radius;
}HoughParams;

class trafficSignDetector{
private:
  ros::Publisher sign_pub;
  image_transport::ImageTransport _it;
  image_transport::Subscriber image_sub;
  ros::ServiceServer param_set;
  cv_bridge::CvImagePtr cv_ptr;
  HoughParams params;
  ros::NodeHandle nh;
  Mat camera_image;

public:
  int hd;
  int sd;
  int vd;

  //parameter for Hough Circle
  
  int color_threshold;
  map<string, Scalar> colors;
  
  trafficSignDetector(ros::NodeHandle nh);
  void imageCB(const sensor_msgs::ImageConstPtr& msg);
  bool set_parameter(opencv_test::hough_parameter::Request &req, opencv_test::hough_parameter::Response &res);
  int detect_color(Mat& src_bgr, Mat& result, Scalar color_bgr) const;
  bool detect_by_color(Mat& src, string color) const;
  int detect(Mat& src) const;
  void start();
};

#endif
