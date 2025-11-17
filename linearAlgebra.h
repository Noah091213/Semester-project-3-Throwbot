#pragma once
#include <vector>

class linearAlgebra {
public:
    std::vector<double> transformPosition(
        std::vector<double> position,
        std::vector<std::vector<double>> transformationMatrix
    );

    std::vector<std::vector<double>> invertTransformationMatrix(
        std::vector<std::vector<double>> matrix
    );

private:
    std::vector<std::vector<double>> eulerToRotationMatrix(double rx, double ry, double rz);
    std::vector<double> rotationMatrixToEuler(const std::vector<std::vector<double>>& R);
};
