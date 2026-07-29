#include "math/matrix.hpp"

vector<double> forwardKinematics(vector<double> lengths, vector<double> angles) {
    vector<double> target;
    Matrix T_res = Matrix::homogeniousMatrix(0.0, angles[0]);

    for(int i = 0; i < lengths.size(); i++) {
        for(int j = 1; j < angles.size(); j++) {
            T_res = T_res * Matrix::homogeniousMatrix(lengths[i], angles[j]);
        }
    }

    int matrixSize = T_res.getValue().size();
    vector<int> origin = {0, 0, 1};
    for(int i = 0; i < matrixSize; i++) {
        target.push_back(T_res.getValue()[i][0] * origin[0] + T_res.getValue()[i][0] * origin[1] + T_res.getValue()[i][0] * origin[2]);
    }

    return target;
}