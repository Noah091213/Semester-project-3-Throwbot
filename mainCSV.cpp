#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

#include "robotControl.h"
#include "matlabCSV.cpp"

int main() {

    robotControl robot;

    std::vector<std::vector<double>> throwOutputs;
    std::vector<double> qStart = {-1.5708, -0.8727, -1.5708, -0.3491, 1.5708, 0};
    matlabCSV matlabComm("/mnt/c/sdu-ubuntu/throwTest/shared.csv");
    int status = 0;

    while (true) {
        std::string input;
        std::cout << std::endl;
        std::cout << "Options Menu:" << std::endl;
        std::cout << " - home: Move robot to home position" << std::endl;
        std::cout << " - position: Get current tool position" << std::endl;
        std::cout << " - grip: Close the gripper" << std::endl;
        std::cout << " - open: Open the gripper" << std::endl;
        std::cout << " - pickup : Pick up a ball" << std::endl;
        std::cout << " - throw: Compute and execute a throw motion" << std::endl;
        std::cout << " - exit: Exit the program" << std::endl;
        std::cout << "What do you want to do?: ";
        std::cin >> input;
        std::cout << std::endl;

        if (input == "home") {
            robot.home();

        } else if (input == "position") {
            robot.getToolPosition();

        } else if (input == "grip") {
            Gripper::grip();

        } else if (input == "open") {
            Gripper::home();

        } else if (input == "pickup") {
            robot.ballPickup();

        } else if (input == "throw") {

            std::cout << "Please enter Release Position (x y z): ";
            double x, y, z;
            std::cin >> x >> y >> z;
            std::vector<double> releasePosition = {x, y, z};
            std::cout << "Please enter yaw, pitch and release velocity: ";
            double yaw, pitch, releaseVelocity;
            std::cin >> yaw >> pitch >> releaseVelocity;
            std::cout << "Please enter lead time and follow time: ";
            double leadTime;
            double followTime;
            std::cin >> leadTime >> followTime;

            std::cout << "Your inputs:" << std::endl;
            std::cout << " Release Position: (" << releasePosition[0] << ", " << releasePosition[1] << ", " << releasePosition[2] << ")" << std::endl;
            std::cout << " Yaw: " << yaw << " Pitch: " << pitch << " Release Velocity: " << releaseVelocity << std::endl;
            std::cout << " Lead Time: " << leadTime << " Follow Time: " << followTime << std::endl;

            throwOutputs = matlabComm.computeThrow(
            status,
            releasePosition,  // releasePosition
            yaw,              // yaw
            pitch,              // pitch
            releaseVelocity,               // releaseVelocity
            leadTime,              // leadTime
            followTime              // followTime
            );
            if (status == 1) {
                std::cout << "Received " << throwOutputs.size() << " output vectors from MATLAB:" << std::endl;
            } else {
                std::cout << "Error in MATLAB computation, status code: " << status << std::endl;
            }

            std::cout << "Execute throw (y/n)?: ";
            std::cin >> input;
            if (input != "y") {
                std::cout << "Throw cancelled." << std::endl;
                continue;
            }

            robot.throwing(throwOutputs, qStart, 0.008, followTime);

        } else if (input == "exit") {
            std::cout << "Exiting program." << std::endl;
            matlabComm.exit();
            break;

        } else {
            std::cout << "Invalid input. Please enter available option (e.g. home or exit)" << std::endl;
        }

    }

    return 0;
}