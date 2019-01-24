#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <detectObstacle.h>
#include <iostream>
using namespace cv;
using namespace std;
DetectObstacle::DetectObstacle(){
    feature[0].minArea = 2200;
    feature[0].maxArea = 10000;
    feature[0].minHSV = Scalar(10,0,98);
    feature[0].maxHSV = Scalar(86,17,212);
    feature[0].minRatio = 0.6;
    feature[0].maxRatio = 0.95;

    feature[1].maxArea = 100000;
    feature[1].minArea = 8000;
    feature[1].minHSV = Scalar(93,13,37);
    feature[1].maxHSV = Scalar(114, 83, 166);
    feature[1].minRatio = 0;
    feature[1].maxRatio = 2.5 ;

    feature[2].maxArea = 2500;
    feature[2].minArea = 7000;
    feature[2].minHSV = Scalar(96,0,35);
    feature[2].maxHSV = Scalar(115, 87, 73);
    feature[2].minRatio = 1.1;
    feature[2].maxRatio = 2.5;

    feature[3].maxArea = 3000;
    feature[3].minArea = 7000;
    feature[3].minHSV = Scalar(96,0,35);
    feature[3].maxHSV = Scalar(127, 52, 67);
    feature[3].minRatio = 0.9;
    feature[3].maxRatio = 1.5;
}
void DetectObstacle::morphos(Mat &thresh)
{

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

    erode(thresh, thresh, erodeElement);


    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);

}
void DetectObstacle::trackObject1 (Mat threshold, Mat &src, ObstacleFeature &f)
{
    Mat threshold_Clone;
    threshold.copyTo(threshold_Clone);

    vector< vector<Point> > listContours;
    vector<Point> contour;

    findContours(threshold_Clone, listContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    
    for (int i = 0; i < listContours.size(); i++)
    {
        Rect border = boundingRect ( listContours[i] );
        double ratio = (double)border.height / border.width;
        double area = border.height * border.width;
        if (ratio >= f.minRatio && ratio <= f.maxRatio && area >= f.minArea && area <= f.maxArea)
        {
            rectangle(src, border, Scalar(0, 0, 255), 1);
        }
    }

}
void DetectObstacle::update(Mat &src){
    Mat HSV;
    cvtColor(src, HSV, CV_BGR2HSV, 0);
    for(int i = 0; i < 4; i++){
        Mat Threshold;
        inRange(HSV,feature[i].minHSV,feature[i].maxHSV, Threshold);
        if (useMorphops)
        {
            morphos(Threshold);
        }
        trackObject1(Threshold, src, feature[i]);
    }
}
