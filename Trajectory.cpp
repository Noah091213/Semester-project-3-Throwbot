#include "Trajectory.h"

/*
CALCTRAJECTORY USAGE EXAMPLE:
TrajectoryResult traj = Vision::calcTrajectory(worldReleasePos, worldTarget);

if (traj.hasLow)
{
    cout << "Low arc:\n";
    cout << " Velocity: " << traj.lowArc.velocity << "\n";
    cout << " Yaw:   " << traj.lowArc.horizAngle << "\n";
    cout << " Pitch: " << traj.lowArc.vertAngle << "\n";
}

if (traj.hasHigh)
{
    cout << "High arc:\n";
    cout << " Velocity: " << traj.highArc.velocity << "\n";
    cout << " Yaw:   " << traj.highArc.horizAngle << "\n";
    cout << " Pitch: " << traj.highArc.vertAngle << "\n";
}

if (!traj.hasLow && !traj.hasHigh)
    cout << "No solution\n";

ANGLES MEASURED FROM X-AXIS:
        +Z (up)
         |
         |  pitch +
         |
         o------> +X (0° yaw)
        /
       /
  +Y
 yaw +

From x towards y is positive yaw, from x towards z is positive pitch.
*/
TrajResult Trajectory::trajMinVelocity(const Vec& worldReleasePos, const Vec& worldTarget) {
    TrajResult result{};
    result.hasSetAngle = false;

    const double g = 9810.0; // mm/s²

    double dx = worldTarget.x - worldReleasePos.x;
    double dy = worldTarget.y - worldReleasePos.y;

    // Horizontal distance on the XY plane
    double horizDist = sqrt(dx*dx + dy*dy);

    // Horizontal angle (yaw)
    double yaw = atan2(dy, dx);

    // Relative vertical distance input for formula
    double h = worldReleasePos.z - worldTarget.z;

    // Straight-line distance from start to target
    double R = sqrt(horizDist*horizDist + h*h);

    // Minimum speed to reach target
    double vMin = sqrt(g * (R - h));
    if (vMin <= 0 || isnan(vMin)) return result;

    double v = vMin;
    double v2 = v * v;

    // Discriminant for pitch angles
    double D = v2*v2 - 2.0*g*v2*h + g*g*horizDist*horizDist;
    if (D < 0) return result;

    double rootD = sqrt(D);

    // Tan of pitch angles (vertical angles)
    double tanLow  = (-v2 - rootD) / (g * horizDist);
    double tanHigh = (-v2 + rootD) / (g * horizDist);

    double pitchLow  = atan(tanLow);
    double pitchHigh = atan(tanHigh);

    result.hasLow  = isfinite(pitchLow);
    result.hasHigh = isfinite(pitchHigh);

    if (result.hasLow)
    {
        result.lowArc.velocity = v;
        result.lowArc.yaw = yaw;
        result.lowArc.pitch  = pitchLow;
    }
    if (result.hasHigh)
    {
        result.highArc.velocity = v;
        result.highArc.yaw = yaw;
        result.highArc.pitch  = pitchHigh;
    }

    return result;
}

TrajResult Trajectory::trajSetAngle(double pitch, const Vec& worldReleasePos, const Vec& worldTarget) {
    TrajResult result{};
    result.hasHigh = false;
    result.hasLow = false;

    result.hasSetAngle = false;

    const double g = 9810.0; // mm/s²

    double dx = worldTarget.x - worldReleasePos.x;
    double dy = worldTarget.y - worldReleasePos.y;

    // Horizontal distance on XY plane
    double horizDist = sqrt(dx*dx + dy*dy);

    // If horizontal distance is zero, impossible for ballistic solution unless same height & downwards pitch
    if (horizDist < 1e-9) return result;

    // Horizontal angle
    double yaw = atan2(dy, dx);

    // Vertical difference
    double h = worldReleasePos.z - worldTarget.z;

    // Precompute trig
    double cosPitch = cos(pitch);

    // Denominator term of v^2 formula
    double denom = (h + horizDist * tan(pitch));

    // If denom <= 0, no real velocity solution
    if (denom <= 0.0) return result;

    double v2 = (g * horizDist * horizDist) / (2.0 * cosPitch * cosPitch * denom);

    if (v2 <= 0.0 || isnan(v2) || isinf(v2)) return result;

    double v = sqrt(v2);

    result.hasSetAngle = true;
    result.setAngle.velocity = v;
    result.setAngle.yaw = yaw;
    result.setAngle.pitch = pitch;

    return result;
}

Vec Trajectory::worldToBase(const Vec& worldVec) {
    // REMEMBER TO SET ACTUAL TRANSFORMATION
    cv::Mat R = (cv::Mat_<double>(3,3) <<
                 0, -1,  0,
                -1,  0,  0,
                 0,  0, -1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 1200, 800, 0);

    Vec P;

    cv::Mat pt = (cv::Mat_<double>(3,1) << worldVec.x, worldVec.y, worldVec.z);
    cv::Mat baseVec = R * pt + t;

    P.x = baseVec.at<double>(0,0);
    P.y = baseVec.at<double>(1,0);
    P.z = baseVec.at<double>(2,0);

    return P;
}

Vec Trajectory::baseToWorld(const Vec& baseVec) {
    // REMEMBER TO SET ACTUAL TRANSFORMATION
    cv::Mat R = (cv::Mat_<double>(3,3) <<
                 0, -1,  0,
                -1,  0,  0,
                 0,  0, -1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 1200, 800, 0);

    Vec P;

    cv::Mat pt = (cv::Mat_<double>(3,1) << baseVec.x, baseVec.y, baseVec.z);
    cv::Mat worldVec = R * pt + t;

    P.x = worldVec.at<double>(0,0);
    P.y = worldVec.at<double>(1,0);
    P.z = worldVec.at<double>(2,0);

    return P;
}
