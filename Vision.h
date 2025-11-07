#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include "Trajectory.h"

using namespace std;

class Vision {
public:
    // tableID exists to create seperate .yaml files for each calibration at each table
    static void calibrateCam(int BoardWidth, int boardHeigth, double squareSize, int calibrationAmount, int tableID); // Only inner corners, in millimeters
    static void calibrateTableCorners(const cv::Mat& undistortedImage, int tableID); // Remember to click corners in correct order

    static cv::Mat grabSingleImage(); // Raw distorted image from camera
    static cv::Mat undistortImage(const cv::Mat& distortedImgage, int tableID);
    static cv::Mat rectifyImage(const cv::Mat& undistortedImgage, int tableID);

    static Vec findCircularObject(const cv::Mat& rectifiedImage, double cannyThresh, double circleConfidence, int minRadius, int maxRadius); // Table frame
    static Vec tableToWorld(const Vec& tableVec);
};

#endif
