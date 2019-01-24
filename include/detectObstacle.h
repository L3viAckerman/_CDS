#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#ifndef DETECT_OBSTACLE_H
#define DETECT_OBSTACLE_H
struct ObstacleFeature{
    float minRatio;
    float maxRatio;
    int minArea;
    int maxArea;
    Scalar minHSV;
    Scalar maxHSV;
};
class DetectObstacle 
{
    public:
        DetectObstacle();
        ~DetectObstacle();
        void update(Mat &src);
    private:
        bool useMorphops = true;
        ObstacleFeature feature[4];
        void morphos(Mat &thresh);
        void trackObject1 (Mat threshold, Mat &src, ObstacleFeature &f );
};
#endif
