#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "robotControl.h"
#include "dataLog.h"


using namespace ur_rtde;

    // Constructor (connects automatically)
    robotControl::robotControl(std::string ip, std::vector<std::vector<double>> transformW2R) {

        std::cout << "[Robot Info] Initializing robotControl..." << std::endl;
    
        robot_ip = ip;
        this->transformW2R = transformW2R;

        try {
            rtde_control = std::make_unique<RTDEControlInterface>(robot_ip);
            rtde_receive = std::make_unique<RTDEReceiveInterface>(robot_ip);
            std::cout << "[Robot Info] Connected to UR robot at " << robot_ip << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Robot Error] Failed to connect to robot: " << e.what() << std::endl;
            rtde_control = nullptr;
            rtde_receive = nullptr;
        }

        if (rtde_control && rtde_receive) {
            std::cout << "[Robot Info] RTDE interfaces initialized." << std::endl;
        }

        Gripper::connectGripper();

    } 

    // Destructor (automatically disconnects if still connected)
    robotControl::~robotControl() {
        Gripper::bye();
        disconnect();
    }

    // Manually disconnect
    void robotControl::disconnect() {
        if (rtde_control) {
            try {
                rtde_control->stopScript();  // Stop any running program
                rtde_control.reset();        // Cleanly destroy the control interface
                rtde_receive.reset();        // Cleanly destroy the receive interface
                std::cout << "[Robot Info] Disconnected from robot." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[Robot Error] Error during disconnect: " << e.what() << std::endl;
            }
        }
    }

    // Throw command (vector of vectors)
    void robotControl::throwing(const std::vector<std::vector<double>>& qd, std::vector<double> qStart, double dt, double followTime, std::string fileName) {
        if (!rtde_control) {
            throw std::runtime_error("[Robot Error] SpeedJ failed. Not connected to robot.");
        }

        std::cout << "[Robot Info] Preparing to throw..." << std::endl;
        std::cout << "[Robot Info] Moving to starting position." << std::endl;

        rtde_control->moveJ(qStart, 0.5, 0.5);

        std::cout << "[Robot Info] Starting position reached. Executing throw in 500ms." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "[Robot Info] Sending " << qd.size() << " speedJ commands." << std::endl;
    
        double a = 7.5; // acceleration
        int count = 0; // command counter
        int releaseIndex = qd.size() - static_cast<int>(round(followTime * freq)); // index to release gripper

        std::vector<std::vector<double>> qLog;
        std::vector<std::vector<double>> qdLog;

        for (const auto& speeds : qd) {
            auto start = std::chrono::high_resolution_clock::now();

            std::vector<double> qCurrent = rtde_receive->getActualQ();
            std::vector<double> qdCurrent = rtde_receive->getActualQd();

            // Log q and qd
            qLog.push_back(qCurrent);
            qdLog.push_back(qdCurrent);

            // Print joint positions
            if (count % 10 == 0) {
                std::cout << "[Robot Info] Actual q " << count << ": ";
                for (int i = 0; i < qCurrent.size(); ++i) {
                    std::cout << (qCurrent[i] * (180/M_PI)) << " ";
                }
                std::cout << std::endl;
            } else if (count > releaseIndex - 10 && count < releaseIndex + 10) {
                std::cout << "[Robot Info] Actual q " << count << ": ";
                for (int i = 0; i < qCurrent.size(); ++i) {
                    std::cout << (qCurrent[i] * (180/M_PI)) << " ";
                }
                std::cout << std::endl;
            }

            // Release point
            if (count == releaseIndex) {
                std::vector<double> q = rtde_receive->getActualQ();
                std::cout << "[Robot Info] Release at q " << count << ": ";
                for (int i = 0; i < q.size(); ++i) {
                    std::cout << (q[i] * (180/M_PI)) << " ";
                }
                std::vector<double> qdRelease = rtde_receive->getActualQd();
                std::cout << "[Robot Info] qd at q " << count << ": ";
                for (int i = 0; i < qdRelease.size(); ++i) {
                    std::cout << (qdRelease[i] * (180/M_PI)) << " ";
                }
                std::cout << std::endl;
                getToolPosition();
            }
            if (count == releaseIndex - 5) {
                std::thread([]() {
                    Gripper::release();  // runs independently
                }).detach();
            }

            rtde_control->speedJ(speeds, a, dt - 0.002);
            count++;

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, dt - elapsed.count())));

        }

        rtde_control->speedStop(5.0);   // 5 rad/s deceleration

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        home();

        while(true) {
            std::cout << "Do you want to save the data? (y/n)" << std::endl;
            std::string userInput;
            std::cin >> userInput;

            if (userInput == "y" || userInput == "Y") {

                std::cout << "Saving data in " << fileName << std::endl;
                dataLog::logData(qLog, qdLog, fileName);
                break;

            } else if (userInput == "n" || userInput == "N"){

                std::cout << "Discarding data..." << std::endl;
                break;
                
            } else {
                std::cout << "Please input either y or n..." << std::endl;
            }
        }

    }

    // Send single speedJ command (vector)
    void robotControl::speedJ(const std::vector<double>& qd, double a, double t){
        if (!rtde_control) {
            throw std::runtime_error("[Robot Error] SpeedJ failed. Not connected to robot.");
        }

        rtde_control->speedJ(qd, a, t);

        rtde_control->stopJ(5.0);   // 5 rad/s deceleration

    }
    
    // Pick up ball
    void robotControl::ballPickup(){
        std::cout << "[Robot Info] Picking up ball..." << std::endl;
        rtde_control->moveJ({-0.5524, -1.3533, -1.7579, -1.5160, 1.5622, 0.0012}, 0.5, 0.5);
        Gripper::home();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rtde_control->moveJ({-0.5524, -1.3678, -1.9516, -1.3081, 1.5622, 0.0012}, 0.2, 0.2);
        Gripper::grip();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rtde_control->moveJ({-0.5524, -1.3533, -1.7579, -1.5160, 1.5622, 0.0012}, 0.2, 0.2);
        std::cout << "[Robot Info] Ball picked up." << std::endl;
    }

    // MoveJ to joint position
    void robotControl::moveJ(const std::vector<double>& q, double v, double a, bool wait) {
        if (!rtde_control) {
            throw std::runtime_error("[Robot Error] MoveJ Failed. Not connected to robot.");
        }

        rtde_control->moveJ(q, v, a, !wait);

    }

    // Move to home position
    void robotControl::home() {
        if (!rtde_control) {
            throw std::runtime_error("[Robot Error] MoveJ Failed. Not connected to robot.");
        }

        std::cout << "[Robot Info] Moving to home position." << std::endl;

        rtde_control->moveJ(homePosition, 1.05, 1.4);
    }

    // Get current joint positions
    std::vector<double> robotControl::getJointPositions() {
        if (!rtde_receive) {
            throw std::runtime_error("[Robot Error] getJointPositions Failed. Not connected to robot.");
        }  
        return rtde_receive->getActualQ();
    } 

    // Get current tool position in world frame
    std::vector<double> robotControl::getToolPosition() {
        if (!rtde_receive) {
            throw std::runtime_error("[Robot Error] getToolPosition Failed. Not connected to robot.");
        }  
        std::vector<double> toolPosR = rtde_receive->getActualTCPPose();
        std::vector<double> toolPosW = linearAlgebra().transformPosition(toolPosR, linearAlgebra().invertTransformationMatrix(transformW2R));
        std::cout << "[Robot Info] Tool Position in World Frame: ";
        for (const auto& val : toolPosW) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
        return toolPosW;
    }
