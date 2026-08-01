#include "../math/matrix.hpp"

// Find the end effector position based on joint lengths and angles
Matriks forwardKinematics(vector<double>& lengths, vector<double>& angles) {
    if(lengths.size() != angles.size()) {
        throw invalid_argument("[ERROR] Data invalid");
    }

    Matriks T_res = Matriks::identity(3);

    for(int i = 0; i < lengths.size(); i++) {
        T_res = T_res * Matriks::homogeniousMatriks(lengths[i], angles[i]);
    }

    Matriks origin(vector<vector<double>>{
        {0},
        {0},
        {1}
    });

    Matriks X = T_res * origin;
    Matriks endEffector(vector<vector<double>>{
        {X.getData()[0][0]},
        {X.getData()[1][0]}
    });

    return endEffector;
}