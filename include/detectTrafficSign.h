#ifndef DETECTTRAFFICSIGN_H
#define DETECTTRAFFICSIGN_H
#include <sstream>
#include <string>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "carcontrol.h"


using namespace cv;
using namespace std;

enum TrafficSignType {
    TYPE_ONE,
    TYPE_TWO,
    TYPE_THREE, 
    TYPE_FOUR
};

const int one = 90 * 90;
const int two = 90 * 95;
const int three = 60 * 60;
const int four = 50 * 50;

enum MaxSignAreaForReg {
    ONE = one,
    TWO = two,
    THREE = three,
    FOUR = four
};

TrafficSignType trafficSignType = TYPE_ONE;

class DetectTrafficSign 
{
    
    public:
        DetectTrafficSign();
        ~DetectTrafficSign();
        void detectTrafficSign(Mat &Img); 
        void setTurnWhat(int turnwhat);
        int getTurnWhat();
        void setTypeTurning(TypeTurning typeofturning);
        Status getStatus();
        void setStatus(Status s);
        static int abcxyz;

    private: 
        void createTrackbars();
        void morphOps(Mat &thresh);
        string type2str(int type);
        int RegconizeTrafficSign(Mat cropImg);
        void trackFilteredTrafficSign(Mat threshold, Mat blur, Mat &cameraFeed, Mat &cropImg, float &radius, Point &center);
        Mat getNen_trackbar(int, void *);
        void on_trackbar(int, void *);
        void checkTrafficSignType();
        

        int H_MIN = 80;
        int H_MAX = 150;
        int S_MIN = 150;
        int S_MAX = 255;
        int V_MIN = 100;
        int V_MAX = 255;
        const int FRAME_WIDTH = 640;
        const int FRAME_HEIGHT = 480;
        const int MAX_NUM_OBJECTS = 50;
        const int MIN_SIGN_AREA = 9 * 9;
        const int MAX_SIGN_AREA = (FRAME_HEIGHT * FRAME_WIDTH * 9) / 1.5;
        const int MIN_SIGN_AREA_FOR_REG = 30 * 30;
        int MAX_SIGN_AREA_FOR_REG;
        const string windowName = "Original Image";
        const string windowName1 = "HSV Image";
        const string windowName2 = "Thresholded Image";
        const string windowName3 = "After Morphological Operations";
        const string trackbarWindowName = "Trackbars";

        int  numOfSeeingTrafficSign = 0;
        int turnWhat = 0;
        Status statusCar = NOSIGN;
        Status preStatusCar;
        TypeTurning typeOfTurning;
        Mat ImgForCheckReset;

};

#endif