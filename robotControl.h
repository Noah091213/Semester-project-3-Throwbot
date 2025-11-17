#pragma once

#include <vector>
#include <memory>
#include <string>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "linearAlgebra.h"
#include "Gripper.h"

class robotControl {
public:
    robotControl(std::string ip = "192.168.100.11",
                 std::vector<std::vector<double>> transformW2R = {{
                     { 0.3839,  0.9234, 0.0000, -0.4980},
                     {-0.9234,  0.3839, 0.0012,  0.0906},
                     { 0.0011, -0.0005, 1.0000, -0.0311},
                     {0, 0, 0, 1}}});
    ~robotControl();

    void disconnect();

    void speedJ(const std::vector<std::vector<double>>& qd, double a, double dt, double leadTime);
    void speedJ(const std::vector<double>& qd, double a, double t);

    void moveJ(const std::vector<double>& worldPosition, double v = 1.05, double a = 1.4, bool wait = true);

    std::vector<double> getJointPositions();
    std::vector<double> getToolPosition();

private:
    int freq = 125;
    std::vector<double> q_min = {-2.79253, -3.14159, -2.53073, -1.74533, 1.13446, -6.28319};
    std::vector<double> q_max = {0.43633, 0.0, 0.0, 1.57080, 1.83260, 6.28319};

    std::vector<std::vector<double>> transformW2R;
    std::string robot_ip;

    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control;
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;

    std::vector<double> getViableIK(std::vector<double> position);
    bool isWithinLimits(std::vector<double> q);
};
