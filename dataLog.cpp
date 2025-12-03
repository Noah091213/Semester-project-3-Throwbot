#include "dataLog.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>

void dataLog::logData(std::vector<std::vector<double>> q, std::vector<std::vector<double>> qd, std::string filename) { 

    // Target rows (1-indexed for the user, 0-indexed for the vector)
    const int target_row_4_index = 3;
    const int target_row_5_index = 4;
    
    // 1. Read the entire file content into memory
    std::vector<std::string> file_lines;
    std::ifstream input_file(filename);

    if (!input_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for reading." << std::endl;
        return;
    }

    std::string line;
    while (std::getline(input_file, line)) {
        file_lines.push_back(line);
    }
    input_file.close();

    // 2. Prepare the new row data
    std::string row4_data = create_csv_row(q);
    std::string row5_data = create_csv_row(qd);

    // 3. Modify the content in memory (Rows 4 and 5)
    
    // Check if Row 4 exists. If not, resize the vector and fill the gap with empty lines.
    if (file_lines.size() <= target_row_4_index) {
        file_lines.resize(target_row_4_index + 1, ""); // Resize to fit row 4, fill gap with ""
    }
    file_lines[target_row_4_index] = row4_data;

    // Check if Row 5 exists. If not, resize the vector.
    if (file_lines.size() <= target_row_5_index) {
        file_lines.resize(target_row_5_index + 1, ""); // Resize to fit row 5, fill gap with ""
    }
    file_lines[target_row_5_index] = row5_data;


    // 4. Write all modified content back to the file (overwriting existing content)
    std::ofstream output_file(filename, std::ios::out | std::ios::trunc); 

    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write every line back, followed by a newline character
    for (const auto& l : file_lines) {
        output_file << l << '\n';
    }

    output_file.close();
    std::cout << "Data successfully logged to rows 4 and 5 of " << filename << " using the robust method." << std::endl;
}

double dataLog::createFileNumber() {
    // Get the current time point as time_t (seconds since epoch)
    std::time_t current_time = std::time(nullptr);
    
    // Convert time_t to local calendar time (struct tm)
    std::tm* local_time = std::localtime(&current_time);
    
    // Extract parts for DDMMHHMMSS
    int day_part = local_time->tm_mday;
    // tm_mon is 0-11, so we add 1 for actual month (1-12)
    int month_part = local_time->tm_mon + 1; 
    int hour_part = local_time->tm_hour;
    int minute_part = local_time->tm_min;
    int second_part = local_time->tm_sec;
    
    // 1. Calculate the 10-digit number using long long to ensure safety
    // Structure: DD * 10^8 + MM * 10^6 + HH * 10^4 + MM * 10^2 + SS
    long long formatted_time_ll = 
          (long long)day_part   * 100000000LL
        + (long long)month_part * 1000000LL
        + (long long)hour_part  * 10000LL
        + (long long)minute_part * 100LL
        + (long long)second_part;
        
    // 2. Cast the result to a double
    return static_cast<double>(formatted_time_ll);
}

std::string dataLog::create_csv_row(const std::vector<std::vector<double>>& data) {
    std::stringstream ss;
    
    // Set general formatting (e.g., 5 decimal places)
    // Adjust precision as needed, but avoid fixed unless strictly necessary for fixed-length binary writing.
    ss << std::setprecision(5); 

    bool first_element = true;
    
    // 1. Outer loop: Iterate through each inner vector (e.g., q[0], q[1], q[2]...)
    for (const auto& inner_vec : data) {
        
        // 2. Inner loop: Iterate through each double in the current inner vector
        for (double val : inner_vec) {
            
            // Add a comma before every element EXCEPT the very first one
            if (!first_element) {
                ss << ",";
            }
            
            ss << val;
            first_element = false;
        }
    }
    
    return ss.str();
}