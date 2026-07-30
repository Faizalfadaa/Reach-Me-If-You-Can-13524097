#include "math/matrix.hpp"

// Find the end effector position based on joint lengths and angles
vector<double> forwardKinematics(vector<double> lengths, vector<double> angles) {
    if(lengths.size() != angles.size()) {
        throw invalid_argument("[ERROR] Data invalid")
    }

    vector<double> endEffector;
    Matrix T_res = Matrix::identity();

    for(int i = 0; i < lengths.size(); i++) {
        T_res = T_res * Matrix::homogeniousMatrix(lengths[i], angles[i]);
    }

    int matrixSize = T_res.getRow();
    vector<int> origin = {0, 0, 1};
    for(int i = 0; i < matrixSize - 1; i++) {
        endEffector.push_back(T_res.getData()[i][0] * origin[0] + T_res.getData()[i][1] * origin[1] + T_res.getData()[i][2] * origin[2]);
    }

    return endEffector;
}