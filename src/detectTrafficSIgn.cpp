#include "detectTrafficSign.h"

DetectTrafficSign::DetectTrafficSign()
{
    createTrackbars();
};
int DetectTrafficSign::abcxyz = 0;

void DetectTrafficSign::setTurnWhat(int turnwhat)
{
    turnWhat = turnwhat;
}
int DetectTrafficSign::getTurnWhat()
{
    return turnWhat;
}

void DetectTrafficSign::setTypeTurning(TypeTurning typeofturning)
{
    typeOfTurning = typeofturning;
}

Status DetectTrafficSign::getStatus()
{
    return statusCar;
}

void DetectTrafficSign::setStatus(Status status)
{
    statusCar = status;
}

void DetectTrafficSign::on_trackbar(int, void *)
{ 

}

string intToString(int number)
{

    std::stringstream ss;
    ss << number;
    return ss.str();
}

void DetectTrafficSign::createTrackbars()
{
    //create window for trackbars

    // namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window
    // char TrackbarName[50];
    // sprintf(TrackbarName, "H_MIN", H_MIN);
    // sprintf(TrackbarName, "H_MAX", H_MAX);
    // sprintf(TrackbarName, "S_MIN", S_MIN);
    // sprintf(TrackbarName, "S_MAX", S_MAX);
    // sprintf(TrackbarName, "V_MIN", V_MIN);
    // sprintf(TrackbarName, "V_MAX", V_MAX);
    // createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX);
    // createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX);
    // createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX);
    // createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX);
    // createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX);
    // createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX);
}

void DetectTrafficSign::morphOps(Mat &thresh)
{

    

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

    dilate(thresh, thresh, dilateElement);
}

string DetectTrafficSign::type2str(int type)
{
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

int DetectTrafficSign::RegconizeTrafficSign(Mat cropImg)
{
    int turnwhat;
    long valLeft = 0;
    long valRight = 0;
    for (int i = cropImg.rows / 2; i < cropImg.rows; i++)
    {
        for (int j = cropImg.cols / 4; j < cropImg.cols / 2; j++)
        {
            valLeft += (int)cropImg.at<uchar>(i, j);
            valRight += (int)cropImg.at<uchar>(i, j + cropImg.cols / 4);
        }
    }
    valLeft > valRight ? turnwhat = -1 : turnwhat = 1;
    return turnwhat;
}

void DetectTrafficSign::trackFilteredTrafficSign(Mat threshold, Mat blur, Mat &cameraFeed, Mat &cropImg, float &radius, Point &center)
{
    Mat temp;
    threshold.copyTo(temp);

    vector<vector<Point>> contours;
    findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    double refArea = 0;
    vector<Point> contours_poly;
    vector<Point> _temp;

    Rect boundRect;
    Point2f _center;
    float _radius;
    bool flag = true;
    int indexMax = -1;

    for (int index = 0; index < contours.size(); index++)
    {
        Rect boundRect_temp;
        boundRect_temp = boundingRect(contours[index]);
        double area = boundRect_temp.width * boundRect_temp.height;
        if (area > MIN_SIGN_AREA && area < MAX_SIGN_AREA && area > refArea && boundRect_temp.width / boundRect_temp.height > 0.9 && boundRect_temp.width / boundRect_temp.height < 1.05)
        {
            indexMax = index;
            refArea = area;
        }
    }
    if (indexMax <= -1)
    {
        return;
    }
    MAX_SIGN_AREA_FOR_REG = typeOfTurning.Area;
    if (refArea > MIN_SIGN_AREA_FOR_REG && refArea < MAX_SIGN_AREA_FOR_REG)
    {
        statusCar = SIGN;
        approxPolyDP(contours[indexMax], contours_poly, 3, true);
        boundRect = boundingRect(contours_poly);
        minEnclosingCircle(contours_poly, _center, _radius);
        radius = _radius / 3;
        center.x = _center.x / 3;
        center.y = _center.y / 3;
        cropImg = blur(boundRect);
        turnWhat = RegconizeTrafficSign(cropImg);
    }
    else if (refArea > MAX_SIGN_AREA_FOR_REG)
    {
        statusCar = TURING;
    }
    
}


void DetectTrafficSign::checkTrafficSignType()
{
    if (trafficSignType == TYPE_ONE)
        MAX_SIGN_AREA_FOR_REG = ONE;
    else if (trafficSignType == TYPE_TWO)
        MAX_SIGN_AREA_FOR_REG = TWO;
    else if (trafficSignType == TYPE_THREE)
        MAX_SIGN_AREA_FOR_REG = THREE;
    else 
        MAX_SIGN_AREA_FOR_REG = FOUR;
}

void DetectTrafficSign::detectTrafficSign(Mat &Img)
{
    bool trackObjects = true;
    bool useMorphOps = true;

    Mat HSV;
    Mat threshold;
    Mat re;
    Mat Gaussian;
    Mat blur;
    Mat crop;

    Point center;
    float radius = 0;
    resize(Img, re, Size(), 3, 3, 1);

    cvtColor(re, HSV, COLOR_BGR2HSV);

    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);

    medianBlur(threshold, blur, 3);
    if (useMorphOps)
    {
        morphOps(threshold);
    }
    if (trackObjects)
    {
        trackFilteredTrafficSign(threshold, blur, re, crop, radius, center);
    }
    circle(Img, center, radius, Scalar(0, 0, 255), 1, 8, 0);
    
}
