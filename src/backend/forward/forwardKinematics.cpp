#include "math/matrix.hpp"

// Find the end effector position based on joint lengths and angles
Matrix forwardKinematics(vector<double>& lengths, vector<double>& angles) {
    if(lengths.size() != angles.size()) {
        throw invalid_argument("[ERROR] Data invalid");
    }

    Matrix T_res = Matrix::identity(lengths.size());

    for(int i = 0; i < lengths.size(); i++) {
        T_res = T_res * Matrix::homogeniousMatrix(lengths[i], angles[i]);
    }

    Matrix origin(vector<vector<double>>{
        {0},
        {0},
        {1}
    });

    Matrix X = T_res * origin;
    Matrix endEffector(vector<vector<double>>{
        {X.getData()[0][0]},
        {X.getData()[1][0]}
    });

    return endEffector;
}