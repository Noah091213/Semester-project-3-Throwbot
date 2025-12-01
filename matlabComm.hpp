#ifndef MATLABCOMM_HPP
#define MATLABCOMM_HPP

#include <vector>

std::vector<double> callMatlab(std::vector<double> input);
std::vector<double> createDataToSend(std::vector<double> releasePos, double yaw, double pitch, double releaseVel, double followTime, double frequency, std::vector<std::vector<double>> transformW2R, double excelName);
std::vector<std::vector<double>> sortMatlabResult (std::vector<double> matlabResult, double& statusCode, std::vector<double>& qStart);
void showSortedData(std::vector<std::vector<double>> input);

#endif