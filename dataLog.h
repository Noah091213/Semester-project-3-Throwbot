#pragma once

#include <string>
#include <vector>

class dataLog {
public:

    static void logData(std::vector<std::vector<double>> q, std::vector<std::vector<double>> qd, std::string filename);

    static double createFileNumber();

private:

    static std::string create_csv_row(const std::vector<std::vector<double>>& data);

};