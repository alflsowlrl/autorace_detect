#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv_test/hough_parameter.h"
#include <std_msgs/Int32.h>
#include "detect_traffic_sign.h"

#define HD 3
#define SD 30
#define VD 30

using namespace cv;
using namespace std;

typedef cv::Point3_<uint8_t> Pixel;

trafficSignDetector::trafficSignDetector(ros::NodeHandle nh):
  nh(nh), _it(nh)
{
  ROS_INFO("initialzation");
  //define color
  colors.insert(make_pair("red", Scalar(0, 0, 255)));
  colors.insert(make_pair("green", Scalar(0, 255, 0)));
  colors.insert(make_pair("yellow", Scalar(19, 255, 255)));
  
  //color parameter
  hd = HD;
  sd = SD;
  vd = VD;
  
  //detect_color threshold
  color_threshold = 10;
  
  params.min_dist = 100;
  params.param1 = 100;
  params.param2 = 15;
  params.min_radius = 1;
  params.max_radius = 20;
  
}
void trafficSignDetector::imageCB(const sensor_msgs::ImageConstPtr& msg){
  
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  if(cv_ptr->image.rows > 0 && cv_ptr->image.cols > 0){
    camera_image = cv_ptr->image;

    //debugging
    //namedWindow("ROI", WINDOW_AUTOSIZE);
    //imshow("ROI", camera_image);
  }

  std_msgs::Int32 sign_msg;
  sign_msg.data = detect(camera_image);
  sign_pub.publish(sign_msg);
}

bool trafficSignDetector::set_parameter(opencv_test::hough_parameter::Request &req, opencv_test::hough_parameter::Response &res){
  params.min_dist = req.min_dist;
  params.param1 = req.param1;
  params.param2 = req.param2;
  params.min_radius = req.min_radius;
  params.max_radius = req.max_radius;

  res.response = true;
  return true;
}

int trafficSignDetector::detect_color(Mat& src_bgr, Mat& result, Scalar color_bgr) const{
	    
  //hsv image
  Mat src_hsv;
  cvtColor(src_bgr, src_hsv, COLOR_BGR2HSV);
  
  //hsv color
  Mat color_hsv(1,1, CV_8UC3, color_bgr);
  cvtColor(color_hsv, color_hsv, COLOR_BGR2HSV);
  Pixel color = color_hsv.at<Pixel>(0,0);
	   
  //number of points
  int ptnum = 0;

  //temporary matrix
  int result_col = cvRound(src_hsv.cols / 3);
  int offset = src_hsv.cols - result_col;
  Mat result_hsv(src_hsv.rows, src_hsv.cols, CV_8UC3, Scalar(0, 0, 0));

  //find pixel of interest in roi 
  for(int i = 0; i < src_hsv.rows; i++){
    for(int j = offset; j < src_hsv.cols;j++){
      Pixel* pixel = src_hsv.ptr<Pixel>(i, j);

      
      if(pixel->x > color.x - hd && pixel->x < color.x + hd
	 && pixel->y > color.y - sd && pixel->y < color.y + sd
	 && pixel->z > color.z - vd && pixel->z < color.z + vd){
	
	ptnum++;
	
	result_hsv.at<Pixel>(i, j) = *pixel;
      }
    }
  }
		 
  //split channel to make gray image
  Mat splited[3];
  split(result_hsv, splited);
  result = splited[2];

  //processed image
  //namedWindow("detected", WINDOW_AUTOSIZE);
  //imshow("detected", result);
  
  return ptnum;
}

bool trafficSignDetector::detect_by_color(Mat& src, string color) const{
  
  //gray image that includes pixel with specific color
  Mat detected;
  if(detect_color(src, detected, colors.find(color)->second) < color_threshold)
    return false;

  
  // Storage for circles
  vector<Vec3f> circles;

  //gaussian blur
  GaussianBlur(detected , detected, Size(9, 9), 2, 2 );

  //detect circle
  HoughCircles(detected, circles, CV_HOUGH_GRADIENT, 1, params.min_dist, params.param1, params.param2, params.min_radius, params.max_radius);
  ROS_INFO("param:(%f, %f, %f, %f, %f)", params.min_dist, params.param1, params.param2, params.min_radius, params.max_radius);
  if(circles.size() < 1)
    return false;

  Mat result(src.rows, src.cols, CV_8UC3, Scalar(0, 0, 0));
  // Draw detected circle as red circles.
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      ROS_INFO("%s(%d, %d, %d)", color.c_str(), center.x, center.y, radius);
      // circle center
      circle( result, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( result, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }
  
  //Show circles
  //namedWindow(color, CV_WINDOW_AUTOSIZE);
  //imshow(color, result );
  
  return true;
}

int trafficSignDetector::detect(Mat& src) const{
  bool detected = false;
  if(detect_by_color(src, "green")){
    ROS_INFO("green_sign");
    detected = true;
    return sign_code::GREEN;
  }
  
  
  if(detect_by_color(src, "yellow")){
    ROS_INFO("yellow_sign");
    detected = true;
    return sign_code::YELLOW;
  }
  
  
  if(detect_by_color(src, "red")){
    ROS_INFO("red_sign");
    detected = true;
    return sign_code::RED;
  }

  if(!detected)
    ROS_INFO("no sign");
  return sign_code::NONE;
}
void trafficSignDetector::start(){
  //ros communication
  param_set = nh.advertiseService("detect_traffic_sign_param", &trafficSignDetector::set_parameter, this);
  image_sub = _it.subscribe("/camera/image", 1,  &trafficSignDetector::imageCB, this);
  sign_pub = nh.advertise<std_msgs::Int32>("detect/sign", 1);
}
  
int main( int argc, char** argv ){
  ros::init(argc, argv, "detect_traffic_sign");
  ros::NodeHandle nh;
  
  trafficSignDetector detector(nh);
  detector.start();
  ros::Duration nap(0.1);
  
  while(ros::ok()){
    ros::spinOnce();
    nap.sleep();
  }
}
