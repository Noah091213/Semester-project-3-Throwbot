#include <vector>
#include <iostream>
#include <cmath>
#include "linearAlgebra.h"

using namespace std;

class linearAlgebra {
public:
    // Transform position (3D or 6D) using a 4x4 transformation matrix
    vector<double> transformPosition(vector<double> position, vector<vector<double>> transformationMatrix) {
        // Input position in frame A, output in frame B, given transformation matrix from A to B
        
        // Safety checks
        if (position.size() != 3 && position.size() != 6) {
            cout << "[Algebra Error] Position vector must have 3 or 6 elements." << endl;
            return position; // Invalid input, return as is
        }
        if (transformationMatrix.size() != 4 || transformationMatrix[0].size() != 4) {
            cout << "[Algebra Error] Transformation matrix must be 4x4." << endl;
            return position; // Invalid input, return as is
        }

        // Result vector
        vector<double> res = {0.0, 0.0, 0.0, 0.0};

        // Build a homogeneous 4D position vector
        vector<double> p = {position[0], position[1], position[2], 1.0};

        // Matrix × Vector
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                res[i] += transformationMatrix[i][j] * p[j];
            }
        }

        // Remove 4th element
        res.pop_back();

        // If 6D, calculate rotation part
        if (position.size() == 6) {
            // Build rotation matrix from Euler angles
            vector<vector<double>> rot = eulerToRotationMatrix(position[3], position[4], position[5]);

            // Apply rotation
            vector<vector<double>> newRot(3, vector<double>(3, 0.0));
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    for (int k = 0; k < 3; ++k) {
                        newRot[i][j] += transformationMatrix[i][k] * rot[k][j];
                    }
                }
            }

            // Convert back to Euler angles
            vector<double> eulerAngles = rotationMatrixToEuler(newRot);

            // Add to result
            res.push_back(eulerAngles[0]);
            res.push_back(eulerAngles[1]);
            res.push_back(eulerAngles[2]);
        }

        return res;
    }

    // Invert a 4x4 transformation matrix
    vector<vector<double>> invertTransformationMatrix(vector<vector<double>> matrix) {
        // Safety check
        if (matrix.size() != 4 || matrix[0].size() != 4) {
            cout << "[Algebra Error] Only 4x4 matrix inversion is supported." << endl;
            return matrix;
        }

        // Extract rotation and translation
        vector<vector<double>> rotation(3, vector<double>(3));
        vector<double> translation(3);
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j)
                rotation[i][j] = matrix[i][j];
            translation[i] = matrix[i][3];
        }

        // Transpose rotation
        vector<vector<double>> rotationT(3, vector<double>(3));
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                rotationT[i][j] = rotation[j][i];

        // New translation
        vector<double> newTranslation(3, 0.0);
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                newTranslation[i] -= rotationT[i][j] * translation[j];

        // Build inverted matrix
        vector<vector<double>> inv(4, vector<double>(4, 0.0));
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j)
                inv[i][j] = rotationT[i][j];
            inv[i][3] = newTranslation[i];
        }
        inv[3][3] = 1.0;

        return inv;
    }

private:
    // Convert Euler angles to rotation matrix
    vector<vector<double>> eulerToRotationMatrix(double rx, double ry, double rz) {
        double cx = cos(rx), sx = sin(rx);
        double cy = cos(ry), sy = sin(ry);
        double cz = cos(rz), sz = sin(rz);

        // Rotation order: Rz * Ry * Rx
        return {
            {cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx},
            {sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx},
            {-sy,   cy*sx,            cy*cx}
        };
    }

    // Convert rotation matrix to Euler angles
    vector<double> rotationMatrixToEuler(const vector<vector<double>>& R) {
        double ry = -asin(R[2][0]);
        double cy = cos(ry);
        double rx, rz;

        if (fabs(cy) > 1e-6) { // normal case
            rx = atan2(R[2][1] / cy, R[2][2] / cy);
            rz = atan2(R[1][0] / cy, R[0][0] / cy);
        } else { // gimbal lock
            rx = 0;
            rz = atan2(-R[0][1], R[1][1]);
        }

        return {rx, ry, rz};
    }

};
