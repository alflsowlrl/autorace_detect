#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <cmath> 
#include <vector>
#include <list>

#define PI 3.14159265
#define R_RESOL 0.01

using namespace cv;

double find_yaw(const geometry_msgs::Pose &pose){
  double w = pose.orientation.w;
  double x = pose.orientation.x;
  double y = pose.orientation.y;
  double z = pose.orientation.z;

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);  
  double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

//radian to degree (0~360) 
int wrap_angle(double radian){
    int degree = int(radian * 180/PI);
    while(degree < 0)
        degree += 360;
    while(degree > 360)
        degree -= 360;

    return degree;
}

//log odds
float log_odds(float p){
    return log(p/(1-p));
}
//inverse of log odds
float inv_log_odds(float l){
    return 1 - 1/(1 + exp(l));
}

class Lds{
private:
    float p_free;
    float p_occ;
    float p_prior;
    float radius;
    int height, width;
    double robot_radius;

    //bias x, y used in transform between world and pixel, unit is pixel
    double bias_x;
    double bias_y;

    //origin of occ_gridmap in world coordinate
    double origin_x;
    double origin_y;

    //robot pos
    double xpos;
    double ypos;
    double heading;

    Mat gridmap;
    float ** occ_gridmap;
    
    ros::NodeHandle nh;
    ros::Subscriber lds_sub;
    ros::Subscriber odom_sub;
public:
    Lds(ros::NodeHandle nh);
    void ldsCB(const sensor_msgs::LaserScan & msg);
    void odomCB(const nav_msgs::Odometry &pose);
    void map_update(const std::vector<float> &lds_range, float max_r);
    Point2i world2pixel(const Point2d &w);
    Point2d pixel2world(const Point2i &p);
    void map_show();
    const std::vector<Point2i>& search_path(Point2i& des);
    ~Lds();
};
Lds::Lds(ros::NodeHandle nh)
    :nh(nh)
{
    p_free = 0.2;
    p_occ = 0.8;
    p_prior = 0.5;
    radius = 0.03;
    
    width = 200;
    height = 200;

    bias_x = 20;
    bias_y = height - 10;

    robot_radius = 0.09;

    gridmap = Mat(width, height, CV_8UC1, Scalar(int(255*p_prior)));

    //occupancy gridmap
    occ_gridmap = new float*[height];
    for(int i = 0; i < height; i++){
        occ_gridmap[i] = new float[width];
        for(int j = 0; j < width; j++)
            occ_gridmap[i][j] = log_odds(p_prior);
    }

    

    lds_sub= nh.subscribe("scan", 1, &Lds::ldsCB, this);
    odom_sub = nh.subscribe("odom", 1, &Lds::odomCB, this);
}
void Lds::ldsCB(const sensor_msgs::LaserScan &msg){
    static int count = 0;
    count++;
    if(count == 3)
        count = 0;
    else
        return;
    
    const std::vector<float> &lds_range = msg.ranges;
    float max_r = msg.range_max;


    map_update(lds_range, max_r);
    map_show();
    //ROS_INFO("processed");
    
}

void Lds::odomCB(const nav_msgs::Odometry &msg){
    static bool set_origin = false;

    xpos = msg.pose.pose.position.x;
    ypos = msg.pose.pose.position.y;
    heading = find_yaw(msg.pose.pose);
    if(!set_origin){
        origin_x = xpos;
        origin_y = ypos; 
        set_origin = true;
    }
    //ROS_INFO("x: %f y: %f heading: %f", xpos, ypos, heading);
}
void Lds::map_show(){
    Mat map(width, height, CV_8UC1, Scalar(0));
    uchar *result = map.data;
    ros::Time start = ros::Time::now();
    for(int i = 0; i < height; i ++)
        for(int j = 0; j < width; j++){
            result[width*i + j] = int(255 * inv_log_odds(occ_gridmap[i][j]));

        }
    ros::Time end = ros::Time::now();
    ros::Duration dur = end - start;
    ROS_INFO("time: %f", dur.toSec());
    
    //robot position in image
    Point2i p = world2pixel(Point2d(xpos, ypos));

    circle(map, p, 3, Scalar(255));

    namedWindow("occ", CV_WINDOW_AUTOSIZE);
    imshow("occ", map);
    waitKey(100);
}
void Lds::map_update(const std::vector<float> &lds_range, float max_r){

    for(int i = 0; i < height; i ++){
        for(int j = 0; j < width; j++){

            //position of each cell in world coordinate
            Point2d w = pixel2world(Point2i(j, i));

            float distance = sqrt(float(pow(w.y - ypos, 2) + pow(w.x - xpos, 2)));
            if(distance < max_r){

                int angle = wrap_angle(atan2(w.y - ypos, w.x - xpos) - heading);
                
                //calculate invese_sensor model
                float inv_sensor = 0;
                if(distance < lds_range[angle] - radius/2)
                    inv_sensor = p_free;
                else if(distance < lds_range[angle] + radius/2)
                    inv_sensor = p_occ;
                else
                    continue;

                //update cell
                occ_gridmap[i][j] += log_odds(inv_sensor) + occ_gridmap[i][j] - log_odds(p_prior);
            }
        }
    }
}


Point2i Lds::world2pixel(const Point2d &w){
    int xp = int((w.x - origin_x)/R_RESOL + bias_x);
    int yp = int((w.y - origin_y)/R_RESOL + bias_y);
    return Point2i(xp, yp);
}
Point2d Lds::pixel2world(const Point2i &p){
    double x = (p.x - bias_x) * R_RESOL + origin_x;
    double y = (p.y - bias_y) * R_RESOL + origin_y;
    return Point2d(x, y);
}

Lds::~Lds(){
    for(int i = 0; i < height; i++)
        delete [] occ_gridmap[i];
    delete [] occ_gridmap;
}

class Pixel{
    int i, j;
    Pixel(int i, int j)
        :i(i), j(j)
        {}
    bool operator=(const Pixel& ref){
        return i == ref.i && j == ref.j;
    }
};




int main(int argc, char ** argv){
    ros::init(argc, argv, "opencv_lds");
    ros::Duration nap(0.1);
    ros::NodeHandle nh;
    Lds lds(nh);

    while(ros::ok()){    
        ros::spinOnce();
        nap.sleep();
    }

    return 0;
}