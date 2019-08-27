#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>

using namespace cv;
typedef cv::Point3_<uint8_t> Pixel;


Mat src;
cv_bridge::CvImagePtr cv_ptr;

void mouse_callback(int event, int x, int y, int flags,void* userdata){
  if(event == 1)
    ROS_INFO("%d, %d", x, y);
}
void imageCB(const sensor_msgs::ImageConstPtr& msg){
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  namedWindow("raw_image", WINDOW_AUTOSIZE);
  imshow("raw_image", cv_ptr->image);
  waitKey(100);
  
  //select points for the perspective transform
  Point2f tsrc[] = {Point2f(118, 111), Point2f(193, 111), Point2f(1, 238), Point2f(320,238)};
  Point2f tdst[] = {Point2f(118, 111), Point2f(193, 111), Point2f(114, 238), Point2f(189, 238)};

  Mat transformMatrix = getPerspectiveTransform(tsrc, tdst);
  Mat new_image;
  warpPerspective(cv_ptr->image, new_image, transformMatrix, Size(cv_ptr->image.cols, cv_ptr->image.rows));

  namedWindow("new_image", WINDOW_AUTOSIZE);
  imshow("new_image", new_image);
  
  if(new_image.rows > 0 && new_image.cols > 0){

    Rect rect(0, new_image.rows - 70, new_image.cols, 70);
    new_image(rect).copyTo(src);

    std::cout<<src.rows<<", "<<src.cols<<std::endl;
    ROS_INFO("image received");
    namedWindow("raw_image_ROI", WINDOW_AUTOSIZE);
    imshow("raw_image_ROI", src);
  }
}

double wrap_angle(double theta){
  while(theta > CV_PI)
      theta -= CV_PI;
  while(theta < -CV_PI)
      theta += CV_PI;

  return theta;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_test");
  ros::NodeHandle nh;
  ros::Duration wnap(0.1);
  ros::Duration nap(0.1);
  int naptime = 100;
  
  image_transport::ImageTransport _it(nh);
  image_transport::Subscriber image_sub = _it.subscribe("/camera/image", 1, imageCB);
  ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("/detect/yaw", 1);
  ros::Publisher y_pub = nh.advertise<std_msgs::Int32>("/detect/y", 1);

  std_msgs::Float64 yaw_msg;
  std_msgs::Int32 y_msg;
  cv_bridge::CvImage out_msg;

  Mat src_hsv, detected_edges;

  //define yellow
  const Mat yellow(1,1, CV_8UC3, Scalar(19, 255, 255));
  Mat yellow_hsv(1,1, CV_8UC3, Scalar(0, 0, 0));
  cvtColor(yellow, yellow_hsv, COLOR_BGR2HSV);
  Pixel yp = yellow_hsv.at<Pixel>(0,0);
  Pixel yp_bgr = yellow.at<Pixel>(0,0);

  //define white
  const Mat white(1,1, CV_8UC3, Scalar(255, 255, 255));
  Mat white_hsv(1,1, CV_8UC3, Scalar(0, 0, 0));
  cvtColor(white, white_hsv, COLOR_BGR2HSV);
  Pixel wp = white_hsv.at<Pixel>(0,0);
  Pixel wp_bgr = white.at<Pixel>(0,0);

  //define red
  const Mat red(1,1, CV_8UC3, Scalar(0, 0, 255));
  Mat red_hsv(1,1, CV_8UC3, Scalar(0, 0, 0));
  cvtColor(red, red_hsv, COLOR_BGR2HSV);
  Pixel rp = red_hsv.at<Pixel>(0,0);
  Pixel rp_bgr = red.at<Pixel>(0,0);

  //theta
  double theta = 0;

  //lane width
  int lane_width = 75;
  
  while(ros::ok()){
    ros::spinOnce();
   
    
    if(src.rows <=0 || src.cols <=0){
      wnap.sleep();
      continue;
    }
    
    cvtColor(src, src_hsv, COLOR_BGR2HSV);

    int hd = 3; int sd = 30; int vd = 30;

    Mat result(src.rows, src.cols, CV_8UC3, Scalar(0,0,0));
    std::vector<Point> midpoints;
    
    for(int i = 0; i < src.rows; i++){
      int middlex_y = 0;
      int middlex_w = 0;
      int n_yellow = 0;
      int n_white = 0;
       
      for(int j = 0; j < src.cols;j++){
	      Pixel* src_ptr = src_hsv.ptr<Pixel>(i, j);
	
        if(src_ptr->x > yp.x - hd && src_ptr->x < yp.x + hd
          && src_ptr->y > yp.y - sd && src_ptr->y < yp.y + sd
          && src_ptr->z > yp.z - vd && src_ptr->z < yp.z + vd){

          middlex_y += j;
          n_yellow++;

          result.at<Pixel>(i, j) = yp_bgr;
        }
        if(src_ptr->x > wp.x - hd && src_ptr->x < wp.x + hd
          && src_ptr->y > wp.y - sd && src_ptr->y < wp.y + sd
          && src_ptr->z > wp.z - vd && src_ptr->z < wp.z + vd){

          middlex_w += j;
          n_white++;
      
          result.at<Pixel>(i, j) = wp_bgr;
        }
      }
      if(n_yellow != 0){
	      middlex_y /= n_yellow;
      }
      if(n_white != 0){
	      middlex_w /= n_white;
      }
      if(n_white != 0 && n_yellow != 0){
        result.at<Pixel>(i, (middlex_w + middlex_y) / 2) = rp_bgr;
        midpoints.push_back(Point((middlex_w + middlex_y) / 2, i));
      }
      else if(n_white != 0){
        if(middlex_w - lane_width / 2 > 0)
          result.at<Pixel>(i, middlex_w - lane_width / 2) = rp_bgr;
        midpoints.push_back(Point(middlex_w - lane_width / 2, i));
      }
      else if(n_yellow != 0){	
	      if(middlex_y + lane_width / 2 < result.cols)
	        result.at<Pixel>(i, middlex_y + lane_width / 2) = rp_bgr;
	      midpoints.push_back(Point(middlex_y + lane_width / 2, i));
      }
    }
    
    if(midpoints.size() > 20){
      double mean_x, mean_y, a, a_num, a_den, b;
      mean_x = mean_y = a_num = a_den = a = b = 0;
      for( size_t i = 0; i < midpoints.size(); i++ )
      {
        mean_x += midpoints[i].x;
        mean_y += midpoints[i].y;
      }
      mean_x /= midpoints.size();
      mean_y /= midpoints.size();
      for( size_t i = 0; i < midpoints.size(); i++ )
      {
        a_num += (midpoints[i].x - mean_x) * (midpoints[i].y - mean_y);
        a_den += pow((midpoints[i].y - mean_y), 2); 
      }

      a = a_num / a_den;
      b = mean_x - mean_y * a;
      theta = -atan(a);
      
      ROS_INFO("a: %f, b:%f theta: %f", a, b, theta * 180 /CV_PI);
    }
    
    namedWindow("trajectory", WINDOW_AUTOSIZE);
    imshow("trajectory", result);
    waitKey(100);
    
    yaw_msg.data = theta;
    yaw_pub.publish(yaw_msg);
    ROS_INFO("yaw: %f", yaw_msg.data);

    if(midpoints.size() > 0){
      y_msg.data = midpoints.back().x;
      y_pub.publish(y_msg);
      ROS_INFO("y: %d", y_msg.data);
    }
  }
  
  return 0;
}
