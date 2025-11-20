#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "robotControl.h"

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



    // Send multiple speedJ commands (vector of vectors)
    void robotControl::speedJ(const std::vector<std::vector<double>>& qd, double a, double dt, double leadTime) {
        if (!rtde_control) {
            throw std::runtime_error("[Robot Error] SpeedJ failed. Not connected to robot.");
        }

        std::cout << "[Robot Info] Sending " << qd.size() << " speedJ commands." << std::endl;
    
        int count = 0;

        for (const auto& speeds : qd) {
            auto start = std::chrono::high_resolution_clock::now();

            rtde_control->speedJ(speeds, a, dt - 0.002);
            count++;

            // Print joint positions
            if (count % 25 == 0) {
                std::vector<double> q = rtde_receive->getActualQ();
                std::cout << "[Robot Info] Actual q " << count << ": ";
                for (int i = 0; i < q.size(); ++i) {
                    std::cout << (q[i] * (180/M_PI)) << " ";
                }
                std::cout << std::endl;
            }

            // Release point
            if (count == leadTime * freq + 1) {
                std::vector<double> q = rtde_receive->getActualQ();
                std::cout << "[Robot Info] Release at q " << count << ": ";
                for (int i = 0; i < q.size(); ++i) {
                    std::cout << (q[i] * (180/M_PI)) << " ";
                }
                std::cout << std::endl;
                std::thread([]() {
                    Gripper::release();  // runs independently
                }).detach();
            }

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, dt - elapsed.count())));

        }

        rtde_control->speedStop(5.0);   // 5 rad/s deceleration

    }



    // Send single speedJ command (vector)
    void robotControl::speedJ(const std::vector<double>& qd, double a, double t){
        if (!rtde_control) {
            throw std::runtime_error("[Robot Error] SpeedJ failed. Not connected to robot.");
        }

        rtde_control->speedJ(qd, a, t);

        rtde_control->stopJ(5.0);   // 5 rad/s deceleration

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



    // Helper: Get viable IK solution within joint limits
    std::vector<double> robotControl::getViableIK(std::vector<double> position) {
        try {
            // Get current joint positions
            std::vector<double> qCurrent = rtde_receive->getActualQ();

            // Compute IK using seed qCurrent
            std::vector<double> qSolution = rtde_control->getInverseKinematics(position, qCurrent);

            // Check joint limits
            if (isWithinLimits(qSolution)) {
                return qSolution;
            } else {
                std::cerr << "[Robot Error] IK solution outside joint limits!" << std::endl;
                return {}; // empty vector = no valid solution
            }
        } catch (const std::exception& e) {
            std::cerr << "[Robot Error] IK computation failed: " << e.what() << std::endl;
            return {};
        }

    }



    // Helper: Check if joint positions are within limits
    bool robotControl::isWithinLimits(std::vector<double> q) {
        for (size_t i = 0; i < q.size(); ++i) {
            if (q[i] < q_min[i] || q[i] > q_max[i]) {
                return false;
            }
        }
        return true;
    }




