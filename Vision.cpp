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
    cv::waitKey(0);
    cv::destroyAllWindows();

    return center;
}

std::vector<double> Vision::tableToWorld(const Vec& tableVec) {
    // 0,0 origin of image is expected to be the opposite table corner of world frame
    // Rotation matrix, rotate 180deg around Z and flip Z (table orientation compared to world)
    cv::Mat R = (cv::Mat_<double>(3,3) <<
                 0, 1, 0,
                 1, 0, 0,
                 0, 0,-1);
    // Translation vector (table dimensions)
    cv::Mat t = (cv::Mat_<double>(3,1) << 0, 0, 0);

    Vec P;

    cv::Mat pt = (cv::Mat_<double>(3,1) << tableVec.x, tableVec.y, tableVec.z);
    cv::Mat worldVec = R * pt + t;

    worldVec.at<double>(0,0) = worldVec.at<double>(0,0)/1000;
    worldVec.at<double>(1,0) = worldVec.at<double>(1,0)/1000;
    worldVec.at<double>(2,0) = worldVec.at<double>(2,0)/1000;

    std::vector<double> output = {worldVec.at<double>(0,0), worldVec.at<double>(1,0), worldVec.at<double>(2,0)};

    cout << "center x table frame: " << std::setprecision(5) << output[0] << endl;
    cout << "center y table frame: " << std::setprecision(5) << output[1] << endl;

    return output;
}
