#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
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
    int tableNumber  = 1;
    int targetTable;
    std::string homographyFile;
    std::string calibrationFile;
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
    std::vector<double> worldReleasePos = {700, 400, 400};  // XYZ in mm
    robotControl robot;                                     // Init robot class



    // Main loop
    while(closeProgram == false) {  // Program runs in statemachine, if the program should close, change bool and it will break out and end
        

        switch (programState) {     

            case 0: // Default menu, menu selection
                std::cout << "\nWhat would you like to do?\n\n" << " 1. Calculate trajectory \n 2. Throw the ball \n 3. Calibrate/settings \n 4. Manual control \n 5. Exit program" << std::endl;

                std::cin >> userInputInt;       // Get user input for menu selection
                programState = userInputInt;    // Go to user selected case
            break;



            case 1: // Calculate trajectory
                
                if (calibrationIsDone != true) { // If calibration is not done, return to menu
                    std::cout << "Calibration is not done, please calibrate first!" << std::endl;
                    programState = 0; // Go to default state
                    break;
                }

                // take a fresh picture of the target, then undistort and rectify it
                imgTest = Vision::grabSingleImage();
                imgTestUndist = Vision::undistortImage(imgTest, tableNumber);
                imgTestRectified = Vision::rectifyImage(imgTestUndist, tableNumber);

                // Find a circular object and calculate the center point to aim for (the center of the target)
                centerTarget = Vision::findCircularObject(imgTestRectified, 50, 30, 70, 80);

                if (centerTarget.x == 0) {  // If a target is not found, center is still 0, thus calculations cannot begin
                    programState = 0;       // Return to default state
                    std::cout << "No target found..." << std::endl;
                    break;
                }
                
                // Find the coordinates for the center target in world frame
                centerTargetWorldFrame = Vision::tableToWorld(centerTarget);

                // Create a datalog file number for matlab and convert it to a string for C++
                excelNumber = dataLog::createFileNumber();
                excelName = "DataLogs/" + std::to_string(static_cast<int>(excelNumber)) + ".csv";
                std::cout << "Excel number: " << std::fixed << std::setprecision(0) << excelNumber << " Excel name: " << excelName << std::endl;

                // Create the vector with all data needed for trajectory planning
                matlabDataToSend = createDataToSend(centerTargetWorldFrame, followTime, frequency, transformW2R, excelNumber);

                for (int i = 0; i<matlabDataToSend.size(); i++) {   // Check that the data is formatted correctly
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
                    /*std::cout << "Startin position is: "; // Check to see the data in terminal, Commented and only used for testing
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

                // Pick up and throw ball
                robot.ballPickup(); 
                robot.throwing(calculatedTrajectory, qStart, 0.008, followTime, excelName);

                programState = 0;
            break;



            case 3: // Calibrate/settings
                std::cout << "\n\nCalibration menu:\n\n" << " 1. Choose a table (default=1) \n 2. Calibrate camera \n 3. Calibrate tabel corners \n 4. Exit to main menu" << std::endl;
                std::cin >> userInputInt;

                switch (userInputInt) {
                    case 1:
                        std::cout << "What table number would you like to set? (1-4)" << std::endl;
                        std::cin >> targetTable;

                        if (targetTable < 1 || targetTable > 4) {
                            targetTable = 1;
                            std::cout << "Please select a number between 1 and 4... Defaulted to 1." << std::endl;
                        }

                        // Construct filenames dynamically
                        homographyFile = "homography_table_" + std::to_string(targetTable) + ".yaml";
                        calibrationFile = "camera_calibration_table_" + std::to_string(targetTable) + ".yaml";

                        // Check if BOTH files exist
                        if (std::filesystem::exists(homographyFile) && std::filesystem::exists(calibrationFile)) {
                            tableNumber = targetTable;
                            std::cout << "Table is now set to: " << tableNumber << std::endl;
                        } else {
                            std::cerr << "Error: Configuration files for table " << targetTable << " do not exist." << std::endl;
                            tableNumber = 1;
                            std::cout << "Defaulted to 1." << std::endl;
                        }

                    break;

                    case 2: // Camera calibration
                        std::cout << "what table number are you calibrating?" << std::endl;
                        std::cin >> userInputInt;
                        std::cout << "You are calibrating for table " << userInputInt << std::endl;
                    
                        Vision::calibrateCam(9, 6 , 35.0, 20, userInputInt);

                    break;
                        
                    case 3: // Homography calibration
                        std::cout << "what table number are you calibrating?" << std::endl;
                        std::cin >> userInputInt;
                        std::cout << "You are calibrating for table " << userInputInt << std::endl;
                        
                        imgTest = Vision::grabSingleImage();
                        imgTestUndist = Vision::undistortImage(imgTest, userInputInt);
                        Vision::calibrateTableCorners(imgTestUndist, userInputInt);
                        
                    break;

                    case 4: // Exit to menu
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
