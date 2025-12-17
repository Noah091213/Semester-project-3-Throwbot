#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"
#include <iostream>
#include <vector>
#include <memory>



std::vector<double> callMatlab(std::vector<double> input) {
    
    std::cout << "Starting matlab script..." << std::endl;
    std::unique_ptr<matlab::engine::MATLABEngine> matlabPtr = matlab::engine::startMATLAB();    // Start the Matlab script process, pointer to this is the matlabPtr
    
    matlab::data::ArrayFactory factory; // All data sent to Matlab has to be in their data format, thus a Matlab data array factory is needed

    std::vector<size_t> dims = {1, static_cast<size_t>(input.size())}; // Matlab engine requires VERY specific data types to interact, so the simple array size cannot compile just with an int

    auto arrayInput = factory.createArray<double>(  // Create the actual array for matlab with dimensions of dims, and all the data in the input vector
        dims,
        input.data(), 
        input.data()+input.size()
    );

    std::vector<matlab::data::Array> functionInput({arrayInput});   // If more data would be sent, storing it in an array is helpful

    matlabPtr->eval(u"addpath('..')");  // Add the path where the script is located
    
    matlab::data::TypedArray<double> result = matlabPtr->feval(u"throwCalc", functionInput);    // Run the script with the input of the Matlab array and return result in an array

    std::cout << "finished matlab script\n" << std::endl;
    std::vector<double> vecResult(result.begin(), result.end());    // Convert Matlab array to C++ vector

    return vecResult;
}



std::vector<double> createDataToSend(std::vector<double> targetPos, double followTime, double frequency, std::vector<std::vector<double>> transformW2R, double excelName) {
    std::vector<double> dataToSend; // Create vector for the data to be sent

    std::cout << "Preparing data..." << std::endl;

    // Add data to vector
    dataToSend.push_back(targetPos[0]);
    dataToSend.push_back(targetPos[1]);
    dataToSend.push_back(targetPos[2]);
    dataToSend.push_back(followTime);
    dataToSend.push_back(frequency);
    
    // Transform matrix is 4x4 matrix so double for loop to add that
    for (int i = 0; i < 4; i++) {
        for (int y = 0; y < 4; y++) {
            dataToSend.push_back(transformW2R[i][y]);
        }
    }

    dataToSend.push_back(excelName);

    std::cout << "Created data succesfully!\n" << std ::endl;

    return dataToSend;
}



std::vector<std::vector<double>> sortMatlabResult (std::vector<double> matlabResult, double& statusCode, std::vector<double>& qStart){
    
    std::cout << "Sorting matlab data..." << std::endl;
    std::vector<std::vector<double>> sortedJointPos;

    statusCode = matlabResult[0];  // Save the status code

    if (matlabResult.size() > 13 && (matlabResult.size()-1)%6 == 0) {

        qStart = { matlabResult[1], matlabResult[2], matlabResult[3], matlabResult[4], matlabResult[5], matlabResult[6] }; // Save the qStart
        
        for (int i = 2; i < (matlabResult.size()/6+1); i++) {   // Loop through the remaining parts of the matlab data vector and store the data for joint positions

            std::vector<double> tmp = { matlabResult[i*6-5], matlabResult[i*6-4], matlabResult[i*6-3],matlabResult[i*6-2], matlabResult[i*6-1], matlabResult[i*6] };
            
            sortedJointPos.push_back(tmp);
        }

        return sortedJointPos;

    } else {
        std::cout << "Result cannot be sorted\n" << std::endl;
        return sortedJointPos;
    }
    
}



void showSortedData(std::vector<std::vector<double>> input) {
    
    std::cout << "\nInput size is: " << input.size() <<"\nError code is: ";
    
    for (int x = 0; x < input.size(); x++) {    // Simple for loop to show all the data in the 2 dimensional sorted vector
        for (int y = 0; y < input[x].size(); y++) {
            std::cout << input[x][y] << std::endl;
        }
        std::cout << "\nNext position!\n" << std::endl;
    }
}
