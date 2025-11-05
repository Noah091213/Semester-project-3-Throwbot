#include "Vision.h"
#include <vector>
#include <iostream>

void Vision::calibrateCam(int BoardWidth, int boardHeigth, double squareSize, int calibrationAmount, int tableID) {
    Pylon::PylonAutoInitTerm autoInitTerm;

    cv::Size patternSize(BoardWidth, boardHeigth);
    vector<vector<cv::Point2f>> imgPoints;
    vector<vector<cv::Point3f>> objPoints;

    // Prepare 3D chessboard points for each view
    vector<cv::Point3f> obj;
    for (int i = 0; i < boardHeigth; i++)
        for (int j = 0; j < BoardWidth; j++)
            obj.emplace_back(j * squareSize, i * squareSize, 0);

    try
    {
        Pylon::CInstantCamera cam(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        cam.Open();

        // Disable auto exposure
        GenApi::CEnumerationPtr exposureAuto(cam.GetNodeMap().GetNode("ExposureAuto"));
        if (GenApi::IsWritable(exposureAuto)) exposureAuto->FromString("Off");

        // Set fixed exposure
        GenApi::CFloatPtr exposure(cam.GetNodeMap().GetNode("ExposureTime"));
        if (exposure.IsValid()) exposure->SetValue(30000);

        Pylon::CImageFormatConverter fmt;
        fmt.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        Pylon::CPylonImage pImg;

        cout << "=== Camera Calibration Mode ===" << endl;
        cout << "Show chessboard and press ENTER to capture (" << calibrationAmount << " needed)" << endl;

        cam.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        int imgHeight;
        int imgWidth;
        for (int i = 0; i < calibrationAmount; )
        {
            cout << "Press ENTER for image " << (i + 1) << "/" << calibrationAmount << endl;
            cin.ignore(numeric_limits<streamsize>::max(), '\n');

            Pylon::CGrabResultPtr res;
            cam.RetrieveResult(5000, res, Pylon::TimeoutHandling_ThrowException);
            fmt.Convert(pImg, res);

            imgHeight = res->GetHeight();
            imgWidth = res->GetWidth();
            cv::Mat frame(imgHeight, imgWidth, CV_8UC3, (uint8_t*)pImg.GetBuffer());
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, patternSize, corners);

            if (!found)
            {
                cout << "Chessboard not found, try again" << endl;
                continue;
            }

            cv::cornerSubPix(gray, corners, {11,11}, {-1,-1}, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

            imgPoints.push_back(corners);
            objPoints.push_back(obj);

            cout << "Captured" << endl;
            i++;
        }

        cam.Close();
        cout << "Calibrating..." << endl;

        cv::Mat cameraMatrix, distCoeffs;
        vector<cv::Mat> rvecs, tvecs;

        cv::Size imageSize(imgWidth, imgHeight);
        cv::calibrateCamera(objPoints, imgPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        cout << "\n=== Calibration Complete ===" << endl;
        cout << "Camera Matrix:\n" << cameraMatrix << endl;
        cout << "Distortion Coeffs:\n" << distCoeffs << endl;

        // Save calibration
        cv::FileStorage fs("camera_calibration_table_" + to_string(tableID) + ".yaml", cv::FileStorage::WRITE);
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();

        cout << "Saved to camera_calibration_table_" + to_string(tableID) + ".yaml" << endl;
    }
    catch (const GenICam::GenericException &e)
    {
        cerr << "Camera error: " << e.GetDescription() << endl;
    }
}

namespace {
    // Global variable for mouse callback
    vector<cv::Point2f> clickedPoints;

    void onMouse(int event, int x, int y, int flags, void* userdata) {
        if (event == cv::EVENT_LBUTTONDOWN && clickedPoints.size() < 4)
        {
            clickedPoints.emplace_back((float)x, (float)y);
            cout << "Point " << clickedPoints.size() << ": (" << x << ", " << y << ")" << endl;
        }
    }
}
void Vision::calibrateTableCorners(const cv::Mat& undistortedImage, int tableID) {
    clickedPoints.clear();

    cv::Mat imgCopy = undistortedImage.clone();
    cv::namedWindow("Select Table Corners", cv::WINDOW_NORMAL);
    cv::imshow("Select Table Corners", imgCopy);
    cv::setMouseCallback("Select Table Corners", onMouse);

    cout << "Click the 4 corners of the table in this order: top-left, top-right, bottom-right, bottom-left." << endl;

    while (clickedPoints.size() < 4)
    {
        cv::Mat temp = imgCopy.clone();
        for (const auto& pt : clickedPoints)
            cv::circle(temp, pt, 5, cv::Scalar(0, 0, 255), cv::FILLED);

        cv::imshow("Select Table Corners", temp);
        cv::waitKey(30);
    }

    cv::destroyWindow("Select Table Corners");

    // Ask user for table dimensions
    double tableWidth, tableHeight;
    cout << "(Width and height between the clicked points)" << endl;
    cout << "Enter real-world width of the table in mm (table frame x-axis): ";
    cin >> tableWidth;
    cout << "Enter real-world height of the table in mm (table frame y-axis): ";
    cin >> tableHeight;

    // Define world coordinates (origin at top-left)
    vector<cv::Point2f> tableCorners = {
        {0.0f, 0.0f},
        {(float)tableWidth, 0.0f},
        {(float)tableWidth, (float)tableHeight},
        {0.0f, (float)tableHeight}
    };

    // Compute homography
    cv::Mat H = cv::findHomography(clickedPoints, tableCorners);

    // Save homography
    cv::FileStorage fs("homography_table_" + to_string(tableID) + ".yaml", cv::FileStorage::WRITE);
    fs << "H" << H;
    fs << "tableWx" << tableWidth;
    fs << "tableHy" << tableWidth;
    fs.release();

    cout << "Table calibration complete. Homography saved to homography_table_" + to_string(tableID) + ".yaml." << endl;
}

cv::Mat Vision::grabSingleImage() {
    cv::Mat image;
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create and open camera
        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        camera.Open();

        // Set up format converter for OpenCV
        Pylon::CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        Pylon::CPylonImage pylonImage;

        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        Pylon::CGrabResultPtr ptrGrabResult;
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded())
        {
            formatConverter.Convert(pylonImage, ptrGrabResult);

            image = cv::Mat(ptrGrabResult->GetHeight(),
                            ptrGrabResult->GetWidth(),
                            CV_8UC3,
                            (uint8_t*)pylonImage.GetBuffer()).clone(); // Clone to keep data after release
        }
        else
        {
            cerr << "Image grab failed: "
                      << ptrGrabResult->GetErrorCode() << " "
                      << ptrGrabResult->GetErrorDescription() << endl;
        }

        camera.Close();
    }
    catch (GenICam::GenericException &e)
    {
        cerr << "Exception: " << e.GetDescription() << endl;
    }

    return image;
}

cv::Mat Vision::undistortImage(const cv::Mat& distortedImage, int tableID) {
    cv::Mat undistorted;

    // Load camera calibration
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("camera_calibration_table_" + to_string(tableID) + ".yaml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Failed to open camera_calibration_table_" + to_string(tableID) + ".yaml!" << endl;
        return undistorted;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    if (cameraMatrix.empty() || distCoeffs.empty())
    {
        cerr << "Camera calibration is empty!" << endl;
        return undistorted;
    }

    // Undistort the image
    cv::undistort(distortedImage, undistorted, cameraMatrix, distCoeffs);

    return undistorted;
}

cv::Mat Vision::rectifyImage(const cv::Mat& undistortedImage, int tableID) {
    cv::FileStorage fs("homography_table_" + to_string(tableID) + ".yaml", cv::FileStorage::READ);
    cv::Mat H;
    fs["H"] >> H;
    fs.release();

    cv::Mat rectified;
    if (!H.empty()) {
        cv::warpPerspective(undistortedImage, rectified, H, undistortedImage.size());
    } else {
        cerr << "Failed to load homography_table_" + to_string(tableID) + ".yaml" << endl;
    }

    return rectified;
}

Vec Vision::findCircularObject(const cv::Mat& rectifiedImage, double cannyThresh, double circleConfidence, int minRadius, int maxRadius) {
    Vec center;
    center.x = 0;
    center.y = 0;
    center.z = 0;
    //cv::Mat rectifiedImage = cv::imread("resources/test_target.png");
    if (rectifiedImage.empty()) return center;

    cv::Mat gray;
    cv::cvtColor(rectifiedImage, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5,5), 2);

    vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/8, cannyThresh, circleConfidence, minRadius, maxRadius);

    if (!circles.empty()) {
        float cx = circles[0][0];
        float cy = circles[0][1];
        float r  = circles[0][2];
        center.x = static_cast<double>(cx);
        center.y = static_cast<double>(cy);

        // Test code for tuning here
        cv::circle(rectifiedImage, cv::Point(cvRound(cx), cvRound(cy)), cvRound(r), cv::Scalar(0,0,255), 2); // Draw circle outline
        cv::circle(rectifiedImage, cv::Point(cvRound(cx), cvRound(cy)), 3, cv::Scalar(0,255,0), -1); // Draw circle center
    }

    // Test code for tuning here
    cv::imwrite("rectifiedImageTestOut.png", rectifiedImage);
    cv::imshow(to_string(cannyThresh) + " " + to_string(circleConfidence) + " " + to_string(minRadius) + " " + to_string(maxRadius), rectifiedImage);
    cout << "center x table frame: " << center.x << endl;
    cout << "center y table frame: " << center.y << endl;
    //cv::waitKey(0);

    return center;
}

/* TABLETOWORLD USAGE EXAMPLE:
// Example: rotate 90 degrees around Z, translate by (100,200,0) mm
cv::Mat R = (cv::Mat_<double>(3,3) <<
                 0, -1, 0,
                 1,  0, 0,
                 0,  0, 1);
cv::Mat t = (cv::Mat_<double>(3,1) << 100, 200, 0);

Vec tableVec = Vision::pixelToWorld(cv::Point2f(500, 300));  // table coordinates
Vec worldVec = Vision::tableToWorld(tableVec, R, t);

cout << "World coordinates: "
          << worldVec.x << ", " << worldVec.y << ", " << worldVec.z << endl;
*/
Vec Vision::tableToWorld(const Vec& tableVec, const cv::Mat& R, const cv::Mat& t)
{
    Vec P;

    if (R.empty() || t.empty() || R.rows != 3 || R.cols != 3 || t.rows != 3 || t.cols != 1)
    {
        cerr << "Invalid rotation or translation matrix!" << endl;
        return P;
    }

    cv::Mat pt = (cv::Mat_<double>(3,1) << tableVec.x, tableVec.y, tableVec.z);
    cv::Mat worldVec = R * pt + t;

    P.x = worldVec.at<double>(0,0);
    P.y = worldVec.at<double>(1,0);
    P.z = worldVec.at<double>(2,0);

    return P;
}

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
TrajectoryResult Vision::trajMinVelocity(const Vec& worldReleasePos, const Vec& worldTarget) {
    TrajectoryResult result{};
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

TrajectoryResult Vision::trajSetAngle(double pitch, const Vec& worldReleasePos, const Vec& worldTarget) {
    TrajectoryResult result{};
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

// Rotate world to plane by yaw (project into plane X,Z)
static inline void rotateIntoPlane(double yaw, const Vec& diffBase, double &outX, double &outZ) {
    double c = cos(yaw);
    double s = sin(yaw);
    // forward axis in plane = (cos(yaw), sin(yaw)) in base XY
    outX = c * diffBase.x + s * diffBase.y; // forward distance in plane
    outZ = diffBase.z;                      // vertical stays as Z
}

TrajectoryResult Vision::trajMinVelocityPlane(const Vec& planeReleasePos, const Vec& worldTarget) {
    TrajectoryResult result{};
    result.hasSetAngle = false;

    const double g = 9810.0; // mm/s²

    // 1) World -> Base frame
    Vec targetBase {
        worldTarget.x - robotBaseInWorld.x,
        worldTarget.y - robotBaseInWorld.y,
        worldTarget.z - robotBaseInWorld.z
    };

    // 2) Shoulder joint position in base frame
    Vec planeOriginBase {
        shoulderForward,      // forward from base
        shoulderLateral,      // lateral from base
        shoulderHeight        // up
    };

    // 3) Release point in base frame = shoulder + planeReleasePos
    Vec releaseBase {
        planeOriginBase.x + planeReleasePos.x,
        planeOriginBase.y + planeReleasePos.y,
        planeOriginBase.z + planeReleasePos.z
    };

    // 4) Compute yaw based on shoulder -> target (we want base to face target)
    double dx = targetBase.x - planeOriginBase.x;
    double dy = targetBase.y - planeOriginBase.y;
    if (abs(dx) < 1e-9 && abs(dy) < 1e-9) return result;
    double yaw = atan2(dy, dx);

    // 5) Compute vector from release point to target (in base frame)
    Vec diffBase {
        targetBase.x - releaseBase.x,
        targetBase.y - releaseBase.y,
        targetBase.z - releaseBase.z
    };

    // 6) Rotate into firing plane
    double planeX = 0.0;
    double planeZ = 0.0;
    rotateIntoPlane(yaw, diffBase, planeX, planeZ);

    // We require forward distance positive (cannot shoot backwards)
    if (planeX <= 0.0) return result;

    // 7) Solve minimum velocity ballistic (2D) with horiz = planeX, vertical diff = -planeZ? careful:
    // Using h = releaseZ - targetZ, here diffBase.z = targetZ - releaseZ, so h = -diffBase.z = releaseZ - targetZ = -planeZ
    double h = -planeZ; // release height - target height
    double d = planeX;

    // Straight-line distance
    double R = sqrt(d*d + h*h);
    double vMin = sqrt(g * (R - h));
    if (!isfinite(vMin) || vMin <= 0.0) return result;
    double v = vMin;
    double v2 = v * v;

    // Discriminant for pitch angles
    double D = v2*v2 - 2.0*g*v2*h + g*g*d*d;
    if (D < 0.0) return result;

    double rootD = sqrt(D);

    double tanLow  = (-v2 - rootD) / (g * d);
    double tanHigh = (-v2 + rootD) / (g * d);

    double pitchLow  = atan(tanLow);
    double pitchHigh = atan(tanHigh);

    if (isfinite(pitchLow)) {
        result.hasLow = true;
        result.lowArc.velocity = v;
        result.lowArc.yaw = yaw;
        result.lowArc.pitch = pitchLow;
    }
    if (isfinite(pitchHigh)) {
        result.hasHigh = true;
        result.highArc.velocity = v;
        result.highArc.yaw = yaw;
        result.highArc.pitch = pitchHigh;
    }

    return result;
}

TrajectoryResult Vision::trajSetAnglePlane(double pitch, const Vec& planeReleasePos, const Vec& worldTarget) {
    TrajectoryResult result{};
    result.hasHigh = false;
    result.hasLow = false;

    result.hasSetAngle = false;

    const double g = 9810.0; // mm/s²

    // 1) World -> Base frame
    Vec targetBase {
        worldTarget.x - robotBaseInWorld.x,
        worldTarget.y - robotBaseInWorld.y,
        worldTarget.z - robotBaseInWorld.z
    };

    // 2) Shoulder joint position in base frame
    Vec planeOriginBase {
        shoulderForward,      // forward from base
        shoulderLateral,      // lateral from base
        shoulderHeight        // up
    };

    // 3) Release point in base frame = shoulder + planeReleasePos
    Vec releaseBase {
        planeOriginBase.x + planeReleasePos.x,
        planeOriginBase.y + planeReleasePos.y,
        planeOriginBase.z + planeReleasePos.z
    };

    // 4) Compute yaw from shoulder -> target
    double dx = targetBase.x - planeOriginBase.x;
    double dy = targetBase.y - planeOriginBase.y;
    if (abs(dx) < 1e-9 && abs(dy) < 1e-9) return result;
    double yaw = atan2(dy, dx);

    // 5) Vector from release to target
    Vec diffBase {
        targetBase.x - releaseBase.x,
        targetBase.y - releaseBase.y,
        targetBase.z - releaseBase.z
    };

    // 6) Rotate into plane
    double planeX = 0.0;
    double planeZ = 0.0;
    rotateIntoPlane(yaw, diffBase, planeX, planeZ);

    if (planeX <= 0.0) return result; // can't shoot backwards

    // h = releaseZ - targetZ = -planeZ (since planeZ = targetZ - releaseZ)
    double h = -planeZ;
    double d = planeX;

    double tanP = tan(pitch);
    double cosP = cos(pitch);
    double cos2 = cosP * cosP;

    // denom = 2 cos^2(theta) (d * tan(theta) - h)
    double denom = 2.0 * cos2 * (d * tanP - h);

    if (denom <= 0.0) return result; // geometry impossible (e.g., pitch too low/high)

    double v2 = (g * d * d) / denom;
    if (!isfinite(v2) || v2 <= 0.0) return result;

    double v = sqrt(v2);

    result.hasSetAngle = true;
    result.setAngle.velocity = v;
    result.setAngle.yaw = yaw;
    result.setAngle.pitch = pitch;

    return result;
}
