#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "Vision.h"
#include "robotControl.h"
#include "matlabComm.hpp"
#include "Trajectory.h"
#include "dataLog.h"

int main() {

    // Setup
    bool closeProgram = false;
    int programState = 0;
    int userInputInt;
    int tableNumber = 1;
    double frequency = 125;     // Hz
    double followTime = 0.5;    // In seconds
    std::vector<std::vector<double>> transformW2R = {
                     {-0.3840, -0.9233, 0.0005,  0.7021},
                     { 0.9233, -0.3840, 0.0018, -0.7111},
                     {-0.0015,  0.0012, 1.0000, -0.0334},
                     { 0,       0,      0,       1}};
    
    double excelNumber = 0;
    std::string excelName = "";
    bool calculationIsDone = false;
    bool calibrationIsDone = true;

    cv::Mat imgTest;
    cv::Mat imgTestUndist;
    cv::Mat imgTestRectified;

    Vec centerTarget;
    std::vector<double> centerTargetWorldFrame;

    TrajResult traj;
    std::vector<double> matlabDataToSend;
    std::vector<double> matlabDataRecieved;

    std::vector<std::vector<double>> calculatedTrajectory;
    double statusCode;
    std::vector<double> qStart; 

    std::vector<double> worldReleasePos = {700, 400, 400}; // XYZ in mm

    robotControl robot;



    // Main loop
    while(closeProgram == false) {
        

        switch (programState) {

            case 0: // Default menu
                std::cout << "\nWhat would you like to do?\n\n" << " 1. Calculate trajectory \n 2. Throw the ball \n 3. Calibrate/settings \n 4. Manual control \n 5. Exit program" << std::endl;

                std::cin >> userInputInt;
                programState = userInputInt;    // Go to user selected case
            break;



            case 1: // Calculate trajectory
                
                if (calibrationIsDone != true) { // If calibration is not done, return to menu
                    std::cout << "Calibration is not done, please calibrate first!" << std::endl;
                    programState = 0;
                    break;
                }

                // take a fresh picture of the target, then undistort and rectify it
                imgTest = Vision::grabSingleImage();
                imgTestUndist = Vision::undistortImage(imgTest, tableNumber);
                imgTestRectified = Vision::rectifyImage(imgTestUndist, tableNumber);

                // Find a circular object and calculate the center point to aim for (the center of the target)
                centerTarget = Vision::findCircularObject(imgTestRectified, 50, 30, 170, 180);

                if (centerTarget.x == 0) {
                    programState = 0;
                    std::cout << "No target found..." << std::endl;
                    break;
                }
                
                // Find the coordinates for the center target in world frame
                centerTargetWorldFrame = Vision::tableToWorld(centerTarget);

                excelNumber = dataLog::createFileNumber();
                excelName = "DataLogs/" + std::to_string(static_cast<int>(excelNumber)) + ".csv";

                std::cout << "Excel number: " << std::fixed << std::setprecision(0) << excelNumber << " Excel name: " << excelName << std::endl;

                // Create the vector with all data needed for trajectory planning
                matlabDataToSend = createDataToSend(centerTargetWorldFrame, followTime, frequency, transformW2R, excelNumber);

                for (int i = 0; i<matlabDataToSend.size(); i++) {
                    std::cout << matlabDataToSend[i] << std::endl;
                }
                // Call the matlab script with prepared data
                matlabDataRecieved = callMatlab(matlabDataToSend);

                // Sort the result from matlab
                calculatedTrajectory = sortMatlabResult(matlabDataRecieved, statusCode, qStart);

                if (statusCode >= 900000000000000) { // Status code will start with 9 if crash occured, otherwise status code will start with 0 or 1
                    std::cout << "A major error occured during calculation..." << std::endl;
                    std::cout << "Error code: " << std::fixed << std::setprecision(0) << statusCode << std::endl;
                } else {
                    calculationIsDone = true;   // Allows throwing the ball
                    std::cout << "Calculation is complete and should work!" << std::endl;
                    std::cout << "Status is: " << statusCode << std::endl;
                    /*std::cout << "Startin position is: ";
                    for (int i = 0; i < qStart.size(); i++){
                        std::cout << qStart[i] << " , ";
                    }
                    std::cout << "\n" << std::endl;*/
                }
                programState = 0;   // Returns to main menu after calculation
            break; 



            case 2: // Throw
                if (calculationIsDone != true){ // If calculation is not done, return to menu
                    std::cout << "A trajectory has not been calculated, please plan the throw before throwing!" << std::endl;
                    programState = 0;
                    break;
                }

                robot.ballPickup();
                robot.throwing(calculatedTrajectory, qStart, 0.008, followTime, excelName);

                programState = 0;
            break;



            case 3: // Calibrate/settings
                std::cout << "\n\nCalibration menu:\n\n" << " 1. Use existing calibration \n 2. Calibrate camera \n 3. Calibrate tabel corners \n 4. Exit to main menu" << std::endl;
                std::cin >> userInputInt;

                switch (userInputInt) {
                    case 1:


                    break;

                    case 2:
                        std::cout << "what table number are you calibrating?" << std::endl;
                        std::cin >> userInputInt;

                        std::cout << "You are calibrating for table " << userInputInt << std::endl;
                    
                        Vision::calibrateCam(9, 6 , 35.0, 20, userInputInt);

                    break;
                        
                    case 3:
                        std::cout << "what table number are you calibrating?" << std::endl;
                        std::cin >> userInputInt;

                        std::cout << "You are calibrating for table " << userInputInt << std::endl;
                        imgTest = Vision::grabSingleImage();
                        imgTestUndist = Vision::undistortImage(imgTest, userInputInt);
                        Vision::calibrateTableCorners(imgTestUndist, userInputInt);
                        
                    break;

                    case 4:
                        programState = 0;

                    break;
                }
            break;



            case 4: // Manual control
                std::cout << "\n\nManual control options:\n\n" << " 1. Home \n 2. Pick up ball \n 3. Open gripper \n 4. Close gripper \n 5. Get TCP location \n 6. Return to main menu" << std::endl;
                std::cin >> userInputInt;
                
                switch (userInputInt) {
                    
                    case 1: // Home
                        robot.home();

                    break;

                    case 2: // Pick up ball
                        robot.ballPickup();

                    break;  

                    case 3: // Open Gripper
                        Gripper::home();

                    break;

                    case 4: // Close gripper
                        Gripper::grip();

                    break;

                    case 5: // Get TCP position
                        robot.getToolPosition();

                    break;

                    case 6: // Exit to main menu
                        programState = 0;

                    break;
                };

            break;



            case 5: // Exit program
                closeProgram = true;
            break;
        };


    }

    return 0;
}



/*
int test() {
    cout << "Starting VISION" << endl;
    //Vision::calibrateCam(8, 6 , 35.0, 20, 1);
    cv::Mat imgTest = Vision::grabSingleImage();
    cv::Mat imgTestUndist = Vision::undistortImage(imgTest, 1);
    //Vision::calibrateTableCorners(imgTest, 1);
    cv::Mat imgTestRectified = Vision::rectifyImage(imgTestUndist, 1);

    Vec centerTarget = Vision::findCircularObject(imgTestRectified, 50, 30, 170, 180);

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
    worldReleasePos.x = 600;
    worldReleasePos.y = 400;
    worldReleasePos.z = 400;

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
}*/
