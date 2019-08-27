#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <opencv_test/coeff.h>
#include <nav_msgs/Odometry.h>
#include <cmath> 
#include <vector>
#include <list>

#define PI 3.14159265
#define R2 1.41421356
#define R_RESOL 0.04

using namespace cv;
using namespace Eigen;

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

class Pixel{
public:
    const int i, j;
    Pixel(int i = -1, int j = -1)
        :i(i), j(j)
        {}
    Pixel(const Pixel& p)
        :i(p.i), j(p.j)
        {}
    bool operator==(const Pixel& ref) const{
        return i == ref.i && j == ref.j;
    }
    bool operator!=(const Pixel& ref) const{
        return !(i == ref.i && j == ref.j);
    }
    double distance_from(const Pixel& p) const{
        return sqrt(pow(i - p.i, 2) + pow(j - p.j, 2));
    }
};

double distance_of_pixels(const Pixel& p1, const Pixel& p2){
    return sqrt(pow(p1.i - p2.i, 2) + pow(p1.j - p2.j, 2));
}

class PosNode{
public:
    Pixel pos;
    const PosNode * parent; 
    double whole_cost;
    double start_cost; //cost from start to here
    PosNode(Pixel pos, const PosNode * parent, double whole_cost, double start_cost)
        :pos(pos), parent(parent), whole_cost(whole_cost), start_cost(start_cost)
        {}
};

class Pathfinder{
public:
    Mat * map_arr;
    uchar *map;
    int width, height;

    std::list<PosNode*> open_list;
    std::list<PosNode*> closed_list;

    Pixel start;
    Pixel goal;
    
    Pathfinder(const Pixel& start, const Pixel& goal, Mat * map_arr, int width, int height)
        :start(start), goal(goal), map_arr(map_arr), width(width), height(height)
    {    
        map = map_arr->data;
    }
    bool valid_index(int i, int j){
        return i > -1 && i < height && 
               j > -1 && j < width && 
               map[i*width + j] < 200;
    }

    double goal_cost(int i, int j){
        return sqrt(pow(i- goal.i, 2) + pow(j - goal.j, 2));
    }
    void append_pos(int i, int j, PosNode * current, double start2here){
        if(current == NULL){
            ROS_INFO("current is NULL");
            return;
        }

        if(valid_index(i, j))
            if(current->parent == NULL || current->parent->pos != Pixel(i, j))
            {
                double whole_cost = goal_cost(i, j) + start2here;
                PosNode * next = new PosNode(Pixel(i, j), current, whole_cost, start2here);
                
                if(open_list.size() == 0)
                    open_list.push_back(next);

                std::list<PosNode*>::iterator iter;
                std::list<PosNode*>::iterator position;

                bool finded = false;
                bool deleted = false;
                bool same = false;

                for (iter = open_list.begin(); iter != open_list.end(); ++iter){

                    if(whole_cost < (*iter)->whole_cost && !finded){
                        position = iter;
                        finded = true;
                    }
                    if((*iter)->pos == Pixel(i, j)){
                        
                        same = true;
                        if((*iter)->whole_cost > whole_cost){
                            iter = open_list.erase(iter);
                            ROS_INFO("same(%d %d)", (*iter)->pos.i, (*iter)->pos.j);
                            iter--;
                            deleted = true;
                        }   
                    }        
                }
                if(!same || deleted)
                    if(finded)
                        open_list.insert(position, next);
                    else 
                        open_list.push_back(next);

            }
    }

    void search_path(std::list<Pixel>& result_path){
        //start.x means i, start.y j
        PosNode * current = new PosNode(start, NULL, goal_cost(start.i, start.j), 0);
        
        closed_list.push_back(current);
        const PosNode * path;

        while(1){
            int i, j;
            double start2here = current->start_cost + 1;
            
            i = current->pos.i - 1;
            j = current->pos.j;
            append_pos(i, j, current, start2here);

            i = current->pos.i + 1;
            j = current->pos.j;
            append_pos(i, j, current, start2here);


            i = current->pos.i;
            j = current->pos.j - 1;
            append_pos(i, j, current, start2here);


            i = current->pos.i;
            j = current->pos.j + 1;
            append_pos(i, j, current, start2here);

            if(open_list.empty() || current == NULL){
                ROS_INFO("searching path failed");
                return;
            }
            current = open_list.front();
            //ROS_INFO("%d %d %f", current->pos.i, current->pos.j, current->whole_cost);
            open_list.pop_front();

            closed_list.push_back(current);
            
            if(current->pos == goal){
                path = current;
                break;
            }
        }

        int cnt = 0;
        while(path != NULL){
            if(cnt % 3 == 0){
                Pixel * p = new Pixel(path->pos);
                result_path.push_front(*p);
            }
            cnt++;
            path = path->parent; 
        }    
    }

    void show_map(){
        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                printf("%d ", map[i*width + j]);
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }
};

class Lds{
private:
    float p_free;
    float p_occ;
    float p_prior;
    float radius;
    int height, width;
    double robot_radius;
    double distance;

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

    //found path
    std::list<Pixel> * result_path;
    Pixel goal;
    Mat map_with_path;
    VectorXd coeffv;

    //occupancy map
    Mat gridmap;
    float ** occ_gridmap;
    
    ros::NodeHandle nh;
    ros::Subscriber lds_sub;
    ros::Subscriber odom_sub;
    ros::Publisher coeff_pub;
public:
    Lds(ros::NodeHandle nh, int width, int heigth);
    void ldsCB(const sensor_msgs::LaserScan & msg);
    void odomCB(const nav_msgs::Odometry &pose);
    void map_update(const std::vector<float> &lds_range, float max_r);
    Point2i world2pixel(const Point2d &w);
    Point2d pixel2world(const Point2i &p);
    void map_show();
    void find_path(Mat& map);
    void show_path();
    void follow_path();
    const std::vector<Point2i>& search_path(Point2i& des);
    ~Lds();
};
Lds::Lds(ros::NodeHandle nh, int width = 50, int height = 50)
    :nh(nh), width(width), height(height), goal(Pixel(5, width - 5))
{
    p_free = 0.2;
    p_occ = 0.8;
    p_prior = 0.5;
    radius = 0.03;

    bias_x = 5;
    bias_y = height - 5;

    robot_radius = 0.09;
    distance = 0;

    gridmap = Mat(width, height, CV_8UC1, Scalar(int(255*p_prior)));

    result_path = NULL;

    //occupancy gridmap
    occ_gridmap = new float*[height];
    for(int i = 0; i < height; i++){
        occ_gridmap[i] = new float[width];
        for(int j = 0; j < width; j++)
            occ_gridmap[i][j] = log_odds(p_prior);
    }

    odom_sub = nh.subscribe("odom", 1, &Lds::odomCB, this);
    coeff_pub = nh.advertise<opencv_test::coeff>("/contorl/coeff", 1);
    
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

    double pastx = xpos;
    double pasty = ypos;
    xpos = msg.pose.pose.position.x;
    ypos = msg.pose.pose.position.y;
    
    heading = find_yaw(msg.pose.pose);
    if(!set_origin){
        origin_x = xpos;
        origin_y = ypos; 
        set_origin = true;
        lds_sub= nh.subscribe("scan", 1, &Lds::ldsCB, this);
    }
    if(set_origin){
        distance += sqrt(pow(pastx - xpos, 2) + pow(pasty - ypos, 2));
    }
    //ROS_INFO("x: %f y: %f heading: %f", xpos, ypos, heading);
}
void Lds::find_path(Mat& map){

    //set goal and start
    Point2i startp = world2pixel(Point2d(xpos, ypos));
    Pixel start_pos = Pixel(startp.y, startp.x);
    ROS_INFO("(%d %d) (%d %d) %f", start_pos.i, start_pos.j, goal.i, goal.j, start_pos.distance_from(goal));

    if(start_pos.distance_from(goal) < 0.1/R_RESOL){
        ROS_INFO("already in goal");
        return;
    }

    if(result_path != NULL)
        delete result_path;
    result_path = new std::list<Pixel>;
    map_with_path = map.clone();
    
    //find path
    Pathfinder finder(start_pos, goal, &map, width, height);
    finder.search_path(*result_path);

}
void Lds::show_path(){
    Point2i robotp = world2pixel(Point2d(xpos, ypos));
    Pixel robot_pos = Pixel(robotp.y, robotp.x);

    Mat map = map_with_path.clone();
    circle(map, robotp, 2, Scalar(255));
    Mat dst;
    resize(map, dst, Size(200, 200), 0, 0, CV_INTER_LINEAR);

    cvtColor(dst, dst, CV_GRAY2BGR);
    //drawing path
    std::list<Pixel>::iterator path_iter = result_path->begin();
    
    for(path_iter = result_path->begin(); path_iter != result_path->end(); path_iter++){
        circle(dst, Point2i(path_iter->j*4, path_iter->i*4), 5, Scalar(0, 0, 255), 2);
    }

    //circle(map_with_path, world2pixel(Point2d(xpos, ypos)), 10, Scalar(255));
    circle(dst, Point2i(result_path->back().j*4, result_path->back().i*4), 5, Scalar(180));

    //approximated path
    for(int x = 0; x < map.cols; x++){
        int y = int(coeffv(0) * pow(x, 2) + coeffv(1) * x + coeffv(2));
        if(y > 0 && y < map.rows)
            circle(dst, Point2i(x*4, y*4), 2, Scalar(0, 255, 255), 1);
    }
    

    namedWindow("path", CV_WINDOW_AUTOSIZE);
    imshow("path", dst);

}
void Lds::follow_path(){
    if(result_path == NULL || result_path->size() == 0)
        return;
    
    Point2i robotp = world2pixel(Point2d(xpos, ypos));
    Pixel robot_pos = Pixel(robotp.y, robotp.x);
    

    if(robot_pos.distance_from(goal) < 0.1/R_RESOL){
        ROS_INFO("in goal");
        return;
    }

    
    std::list<Pixel>::iterator iter = result_path->begin();
    std::list<Pixel>::iterator nearest = result_path->begin();
    double distance = robot_pos.distance_from(*iter);

    int i = 0; int j = 0;
    for( ; iter != result_path->end(); iter++){

        double d = robot_pos.distance_from(*iter);
        if(distance > d){
            nearest = iter;
            distance = d;  
            j = i - 1;
        } 
    }

    Point2d nearp= pixel2world(Point2i(nearest->j, nearest->i));
    double real_distance = sqrt(pow(nearp.y - ypos, 2) + pow(nearp.x - xpos, 2));
    if(real_distance < 0.4){
        j++;
        nearest++;
        nearp = pixel2world(Point2i(nearest->j, nearest->i));
        real_distance = sqrt(pow(nearp.y - ypos, 2) + pow(nearp.x - xpos, 2));
    }
    
    //approxiaton of path
    int num = 6;
    MatrixXd A(num, 3);
    VectorXd b(num, 1);

    iter = nearest;
    for(int i = 0; iter != result_path->end() && i < num; i ++){
        Point2d p = Point2i(iter->j, iter->i);
        A(i, 0) = pow(p.x, 2);
        A(i, 1) = p.x;
        A(i, 2) = 1;

        b(i, 0) = p.y;
        iter++;
    }
    coeffv = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
    double y_d = coeffv(0) * pow(robotp.x, 2) + coeffv(1) * robotp.x + coeffv(2);
    ROS_INFO("diff %f", y_d - robotp.y);
    show_path();
}

void Lds::map_show(){
    Mat map(width, height, CV_8UC1, Scalar(0));
    uchar *result = map.data;
    ros::Time start = ros::Time::now();
    for(int i = 0; i < height; i ++)
        for(int j = 0; j < width; j++){
            result[width*i + j] = int(255 * inv_log_odds(occ_gridmap[i][j]));
            if(result[width*i + j] > 200)
                circle(map, Point2i(j, i), ceil(robot_radius/R_RESOL), Scalar(255) , -1);
        }

    if(distance > 0.2){
        while(distance > 0.2)
            distance -= 0.2;
        find_path(map);
    }
    

    ros::Time end = ros::Time::now();
    ros::Duration dur = end - start;
    //ROS_INFO("time: %f", dur.toSec());
    
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
    if(result_path != NULL)
        delete result_path;

    for(int i = 0; i < height; i++)
        delete [] occ_gridmap[i];
    delete [] occ_gridmap;
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "opencv_lds");
    ros::Duration nap(0.1);
    ros::NodeHandle nh;
    Lds lds(nh);

    while(ros::ok()){    
        ros::spinOnce();
        lds.follow_path();
        nap.sleep();
    }
    return 0;
}