#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

#include "detectlane.h"

using namespace std;
using namespace cv;
enum Status {
        SIGN,
        NOSIGN,
        TURING
};

extern fstream fileLog;
extern string pathOfLogFile;

extern Point CENTER;


struct TypeTurning 
{
    long time;
    int Area;
    int velocity;
    int error;
};

class CarControl 
{
public:
    CarControl();
    ~CarControl();
    float maxAngle = 7;
    float minAngle = 1;
    void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity);
    void setTurnWhat(int turnwhat);
    int getTurnWhat();
    Status getStatus();
    void setStatus(Status s);
    Status getPreStatus();
    void setTypeTurning(TypeTurning typeofturning);

private:
    float errorAngle(const Point &dst);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = 40;

    float minVelocity = 10;
    float maxVelocity = 50;

    float hasTrafficSignVelocity = 40;
    float rotateAngle = 5;

    float preError;

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;


    int turnWhat = 0;
    Status statusCar = NOSIGN;
    Status preStatusCar;
    TypeTurning typeOfTurning;

};

#endif
