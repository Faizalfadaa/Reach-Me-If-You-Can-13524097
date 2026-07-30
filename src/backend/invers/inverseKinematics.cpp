#include "iostream"
#include "vector"
#include "forward\forwardKinematics.cpp"

using namespace std;

const double DEGREE = 0.01;

// Compute Error vector
Matrix error(Matrix target, Matrix endEffector) {
    return Matrix(vector<vector<double>>{
        {target.getData()[0][0] - endEffector.getData()[0][0]},
        {target.getData()[1][0] - endEffector.getData()[1][0]}
    });
};

// Create the Jacobian Matrix
Matrix jacobianMatrix(vector<Matrix> derivatives) {
    int jointCount = derivatives.size();

    vector<vector<double>> jacobianData(2, vector<double>(jointCount, 0.0));

    for (int i = 0; i < jointCount; i++) {
        jacobianData[0][i] = derivatives[i].getData()[0][0];
        jacobianData[1][i] = derivatives[i].getData()[1][0];
    }

    return Matrix(jacobianData);
};

// Compute alpha for jacobian transpose
double computeAlpha(Matrix J, Matrix errorPos) {
    Matrix eT = errorPos;
    eT.transpose();

    Matrix JT = J;
    JT.transpose();

    Matrix JJT = J * JT;
    Matrix JJTe = JJT * errorPos;

    double numerator = errorPos.getData()[0][0] * JJTe.getData()[0][0] + errorPos.getData()[1][0] * JJTe.getData()[1][0];
    double denominator = JJTe.getData()[0][0] * JJTe.getData()[0][0] + JJTe.getData()[1][0] * JJTe.getData()[1][0];

    if(abs(denominator) < 1e-12) {
        return 0.0;
    }

    return numerator / denominator;
}

// Inverse Kinematics Method : Jacobian transpose
Matrix jacobianTranspose(vector<double>& lengths, vector<double>& angles, Matrix target) {
    // Get the End effector position using inverse kinematics
    vector<Matrix> derivatives;
    int jointCount = lengths.size();
    bool found = false;

    while(!found) {


        // Get the end effector position
        Matrix endEffector = forwardKinematics(lengths, angles); 

        // Compute the error from the end effector position to the target position
        Matrix errorPos = error(target, endEffector);

        for(int i = 0; i < jointCount; i++) {
            Matrix endEffector = forwardKinematics(lengths, angles); 
            angles[i] += DEGREE;

            // Get the end-effector position after the small rotation
            Matrix test = forwardKinematics(lengths, angles); 
            angles[i] -= DEGREE;

            // Approximate the partial derivative of the end effector position
            // with respect to the current joint angle
            Matrix difference = error(test, endEffector);
            derivatives[i] = Matrix(vector<vector<double>>{
                {difference.getData()[0][0] / DEGREE},
                {difference.getData()[1][0] / DEGREE},
            });
        };
        
        // Create the Jacobian Matrix 
        Matrix J = jacobianMatrix(derivatives);
        double a = computeAlpha(J, errorPos);
        J.transpose();

        // Compute the suggested angular change for each joint using J Transpose × error
        // for each joint for each iteration
        Matrix change = J * errorPos;

        // Update each angle using step size (a)
        for(int i = 0; i < jointCount; i++) {
            angles[i] += a * (change.getData()[i][0]);
        };
    }
};