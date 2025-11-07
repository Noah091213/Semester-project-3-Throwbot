#include <opencv2/opencv.hpp>
#include <iostream>
#include "Vision.h"

int main() {
    cout << "Starting VISION" << endl;
    //Vision::calibrateCam(8, 6 , 35.0, 20, 1);
    cv::Mat imgTest = Vision::grabSingleImage();
    cv::Mat imgTestUndist = Vision::undistortImage(imgTest, 1);
    //Vision::calibrateTableCorners(imgTest, 1);
    cv::Mat imgTestRectified = Vision::rectifyImage(imgTestUndist, 1);

    Vec centerTarget = Vision::findCircularObject(imgTestRectified, 100, 10, 30, 50);

    // Rotation matrix (table orientation compared to world)
    cv::Mat R = (cv::Mat_<double>(3,3) <<
                0, -1,  0,
               -1,  0,  0,
                0,  0, -1);
    // Translation vector (table dimensions)
    cv::Mat t = (cv::Mat_<double>(3,1) << 1200, 800, 0);
    // R and t can probably just be made constant as the table frame is always in the same position relative to the world frame
    Vec centerTargetWorldFrame = Vision::tableToWorld(centerTarget);

    cout << "center x world frame: " << centerTargetWorldFrame.x << endl;
    cout << "center y world frame: " << centerTargetWorldFrame.y << endl;

    Vec worldReleasePos; // mm
    worldReleasePos.x = 400;
    worldReleasePos.y = 400;
    worldReleasePos.z = 600;

    TrajResult traj = Trajectory::trajMinVelocity(worldReleasePos, centerTargetWorldFrame);

    if (traj.hasLow)
    {
        cout << "Low arc:\n";
        cout << " Velocity: " << traj.lowArc.velocity << "\n";
        cout << " Yaw:   " << traj.lowArc.yaw << "\n";
        cout << " Pitch: " << traj.lowArc.pitch << "\n";
    }

    if (traj.hasHigh)
    {
        cout << "High arc:\n";
        cout << " Velocity: " << traj.highArc.velocity << "\n";
        cout << " Yaw:   " << traj.highArc.yaw << "\n";
        cout << " Pitch: " << traj.highArc.pitch << "\n";
    }

    cv::waitKey(0);

    return 0;
}
