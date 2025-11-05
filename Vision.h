#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

using namespace std;

struct TrajectorySolution {
    double velocity; // mm/s
    double yaw; // radians (horizontal angle)
    double pitch; // radians (vertical angle)
    // From x towards y is positive yaw, from x towards z is positive pitch.
};
struct TrajectoryResult {
    bool hasLow;
    bool hasHigh;
    bool hasSetAngle;

    TrajectorySolution lowArc;
    TrajectorySolution highArc;
    TrajectorySolution setAngle;
};
struct Vec {
    double x;
    double y;
    double z;
};

class Vision {
public:
    // tableID exists to create seperate .yaml files for each calibration at each table
    static void calibrateCam(int BoardWidth, int boardHeigth, double squareSize, int calibrationAmount, int tableID); // Only inner corners, in millimeters
    static void calibrateTableCorners(const cv::Mat& undistortedImage, int tableID); // Remember to click corners in correct order

    static cv::Mat grabSingleImage(); // Raw distorted image from camera
    static cv::Mat undistortImage(const cv::Mat& distortedImgage, int tableID);
    static cv::Mat rectifyImage(const cv::Mat& undistortedImgage, int tableID);

    static Vec findCircularObject(const cv::Mat& rectifiedImage, double cannyThresh, double circleConfidence, int minRadius, int maxRadius); // Table frame
    static Vec tableToWorld(const Vec& tableVec, const cv::Mat& R, const cv::Mat& t); // Maybe make R and t constant as the frame is always in the same place

    // World point to world point trajectory calculation, struct values are relative to the coordinates
    static TrajectoryResult trajMinVelocity(const Vec& worldReleasePos, const Vec& worldTarget); // World frame, in millimeters
    static TrajectoryResult trajSetAngle(double pitch, const Vec& worldReleasePos, const Vec& worldTarget); // Pitch in radians from x-axis

    // Shooting plane point to world point trajectory calculation, struct values are relative to robot and frame (!)
    // planeReleasePos is given in the virtual plane frame, relative to shoulder joint (2D shooting plane)
    // worldTarget is given in the world frame.
    // planeReleasePos, X is forward, Z is up, Y is 0
    static TrajectoryResult trajMinVelocityPlane(const Vec& planeReleasePos, const Vec& worldTarget);
    static TrajectoryResult trajSetAnglePlane(double pitch, const Vec& planeReleasePos, const Vec& worldTarget);

    // Configuration for the robot base position and shoulder joint offset
    static inline Vec robotBaseInWorld = { 500.0, 500.0, 0.0 }; // robot base in world frame
    // Origin of the 2D shooting plane relative to the robot base
    static inline double shoulderForward = 0.0; // X offset along base forward
    static inline double shoulderLateral = 109.15; // Y offset sideways
    static inline double shoulderHeight = 89.159; // Z height above table
};

#endif
