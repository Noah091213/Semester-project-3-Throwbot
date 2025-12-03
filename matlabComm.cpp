#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"
#include <iostream>
#include <vector>
#include <memory>



std::vector<double> callMatlab(std::vector<double> input) {
    
    std::cout << "Starting matlab script..." << std::endl;
    std::unique_ptr<matlab::engine::MATLABEngine> matlabPtr = matlab::engine::startMATLAB();
    
    matlab::data::ArrayFactory factory;

    std::vector<size_t> dims = {1, static_cast<size_t>(input.size())};

    auto arrayInput = factory.createArray<double>(
        dims,
        input.data(), 
        input.data()+input.size()
    );

    std::vector<matlab::data::Array> functionInput({arrayInput});

    matlabPtr->eval(u"addpath('..')");
    
    matlab::data::TypedArray<double> result = matlabPtr->feval(u"throwCalc", functionInput);

    std::cout << "finished matlab script" << std::endl;
    std::vector<double> vecResult(result.begin(), result.end());

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

    std::cout << "Created data succesfully!" << std ::endl;

    return dataToSend;
}



std::vector<std::vector<double>> sortMatlabResult (std::vector<double> matlabResult, double& statusCode, std::vector<double>& qStart){
    
    std::cout << "Sorting matlab data..." << std::endl;
    std::vector<std::vector<double>> sortedJointPos;

    statusCode = matlabResult[0];  // Save the status code

    if (matlabResult.size() > 13 && (matlabResult.size()-1)%6 == 0) {

        qStart = { matlabResult[1], matlabResult[2], matlabResult[3],matlabResult[4], matlabResult[5], matlabResult[6] }; // Save the qStart

        for (int i = 2; i < (matlabResult.size()/6+1); i++) {   // Loop through the remaining parts of the matlab data vector and store the data for joint positions
            std::cout << "index to sort: " << i << std::endl;

            std::vector<double> tmp = { matlabResult[i*6-5], matlabResult[i*6-4], matlabResult[i*6-3],matlabResult[i*6-2], matlabResult[i*6-1], matlabResult[i*6] };
            
            sortedJointPos.push_back(tmp);
        }

        return sortedJointPos;

    } else {
        std::cout << "Result cannot be sorted" << std::endl;
        return sortedJointPos;
    }
    
}



void showSortedData(std::vector<std::vector<double>> input) {
    
    std::cout << "\nInput size is: " << input.size() <<"\nError code is: ";
    
    for (int x = 0; x < input.size(); x++) {
        for (int y = 0; y < input[x].size(); y++) {
            std::cout << input[x][y] << std::endl;
        }
        std::cout << "\nNext position!" << std::endl;
    }
}


/*
int main() {    // Main is only needed for testing these functions individually. Uncomment and run isolated to test
    
    std::cout << "Beginning setup" << std::endl;
    std::vector<double> releasePosition = {2.0, 2.0, 2.0};
    std::vector<double> qStartPos       = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
    std::vector<std::vector<double>> transformation = { {4.0, 4.0, 4.0, 4.0},
                                                        {5.0, 5.0, 5.0, 5.0},
                                                        {6.0, 6.0, 6.0, 6.0},
                                                        {7.0, 7.0, 7.0, 7.0} };


    std::vector<double> preparedData = createDataToSend(releasePosition, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, qStartPos, transformation);
    
    for (int i = 0; i < preparedData.size(); i++) {
        std::cout << preparedData[i] << std::endl;
    }

    std::vector finishedDataPoints = callMatlab(preparedData);

    for (int i = 0; i < finishedDataPoints.size(); i++) {
        std::cout << finishedDataPoints[i] << std::endl;
    }
    
    std::vector<double> test = {2.2, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
    std::vector<std::vector<double>> sortedTest = sortMatlabResult(test);
    showSortedData(sortedTest);

    return 0;
}*/
