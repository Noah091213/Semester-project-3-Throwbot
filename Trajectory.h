#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <opencv2/opencv.hpp>

using namespace std;

struct TrajSolution {
    double velocity; // mm/s
    double yaw; // radians (horizontal angle)
    double pitch; // radians (vertical angle)
    // From x towards y is positive yaw, from x towards z is positive pitch.
};
struct TrajResult {
    bool hasLow;
    bool hasHigh;
    bool hasSetAngle;

    TrajSolution lowArc;
    TrajSolution highArc;
    TrajSolution setAngle;
};
struct Vec {
    double x;
    double y;
    double z;
};

class Trajectory {
public:
    static TrajResult trajMinVelocity(const Vec& worldReleasePos, const Vec& worldTarget); // World frame, in millimeters
    static TrajResult trajSetAngle(double pitch, const Vec& worldReleasePos, const Vec& worldTarget); // Pitch in radians from x-axis

    static Vec worldToBase(const Vec& worldVec);
    static Vec baseToWorld(const Vec& baseVec);
};

#endif // TRAJECTORY_H
