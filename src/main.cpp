#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include<time.h>

#include "detectlane.h"
#include "carcontrol.h"
#include "detectTrafficSign.h"
#include "detectObstacle.h"


double comp_Hist = 0;

Mat preImg;

double Compare_Hist(Mat &Img, Mat &preImg)
{
    Mat ImgHSV, preImgHSV;
    cvtColor(Img, ImgHSV, CV_BGR2HSV);
    cvtColor(preImg, preImgHSV, CV_BGR2HSV);

    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    int channels[] = { 0, 1 };

    MatND hist_Img, hist_preImg;

    calcHist( &ImgHSV, 1, channels, Mat(), hist_Img, 2, histSize, ranges, true, false );
    normalize( hist_Img, hist_Img, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &preImgHSV, 1, channels, Mat(), hist_preImg, 2, histSize, ranges, true, false );
    normalize( hist_preImg, hist_preImg, 0, 1, NORM_MINMAX, -1, Mat() );

    int compare_method = 1;
    double baseHist = compareHist( hist_Img, hist_preImg, compare_method );
    return baseHist;
}

bool checkRestart()
{
    if ( comp_Hist > 30) return true;
    else return false;
}

const double TIMEFORTURN = 0.10;

bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
DetectTrafficSign *trafficSign;
DetectObstacle *obstacle;


// TypeTurning typeOfTurning[2] = {{1800000, 120 * 125, 30, 6}, {31600000,115 * 125, 35, 7}};
TypeTurning typeOfTurning[2] = {{3000000, 105 * 110, 35, 7}, {2500000, 95 * 95, 30, 7}};


int TYPE_TURN = 0;


double timeTurn = 0;



int skipFrame = 1;

/
int countFrame = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    Mat Img;
    static std::clock_t start;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       
        cv_ptr->image.copyTo(Img);

	    waitKey(1);

      
        if(!preImg.empty())
        {
            comp_Hist = Compare_Hist(Img, preImg);
            cout << "compare Hist " << comp_Hist << endl;
        }

        if (checkRestart()) 
        {
            TYPE_TURN = 0;
            countFrame = 0;
        }

        detect->update(cv_ptr->image);
        car->setTypeTurning(typeOfTurning[TYPE_TURN]);
        trafficSign->setTypeTurning(typeOfTurning[TYPE_TURN]);
        trafficSign->detectTrafficSign(Img);
        car->setStatus(trafficSign->getStatus());
        car->setTurnWhat(trafficSign->getTurnWhat());
        if(car->getStatus() == TURING)
        {
            if(car->getPreStatus() == SIGN)
            {
                start = std::clock();
            } 
            else 
            {
                if(std::clock() - start > typeOfTurning[TYPE_TURN].time)
                {
                    car->setStatus(NOSIGN);
                    trafficSign->setStatus(NOSIGN);
                    TYPE_TURN++;
                    if(TYPE_TURN > 1) 
                        TYPE_TURN = 0;
                }
            }
        }
        
        car->driverCar(detect->getLeftLane(), detect->getRightLane(), 50);
        preImg = Img;
        Mat objDetect = Img.clone();
        obstacle->update(objDetect);
        imshow("obstacle", objDetect);

        countFrame ++;
        cv::imshow("View", Img);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    


    detect = new DetectLane();
    car = new CarControl();
    trafficSign = new DetectTrafficSign();
    obstacle = new DetectObstacle();    

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("team912_image", 1, imageCallback);
        ros::spin();
    } else {
        videoProcess();
    }
    cv::destroyAllWindows();
    
    
    return 0;
}
