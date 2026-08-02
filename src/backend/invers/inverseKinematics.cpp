#include "iostream"
#include "vector"
#include "../forward/forwardKinematics.cpp"
#include "../IKResult.hpp"
#include "../bridge/bridge.hpp"

using namespace std;

#define PI 3.14159265358979323846f
#define DEG2RAD (PI/180.0f) 

// Constants declarations
const double DEGREE = 0.01;
const double MAX_LAMBDA = 5.0;
const double MIN_LAMBDA = 2.0;
const double MAX_DELTA_TETHA = 3.5 * DEG2RAD;
const int MAX_ITERATION = 1000;
const double TOLERANCE = 1.0;

// Compute Error vector
Matriks error(Matriks target, Matriks endEffector) {
    return Matriks(vector<vector<double>>{
        {target.getData()[0][0] - endEffector.getData()[0][0]},
        {target.getData()[1][0] - endEffector.getData()[1][0]}
    });
};

// Create the Jacobian Matriks
Matriks jacobianMatriks(vector<Matriks> derivatives) {
    int jointCount = derivatives.size();

    vector<vector<double>> jacobianData(2, vector<double>(jointCount, 0.0));

    for (int i = 0; i < jointCount; i++) {
        jacobianData[0][i] = derivatives[i].getData()[0][0];
        jacobianData[1][i] = derivatives[i].getData()[1][0];
    }

    return Matriks(jacobianData);
};

// Get all joint positions coordinates
vector<Matriks> getJointPositions(vector<double>& lengths, vector<double>& angles) {
    vector<Matriks> jointPositions;
    double x = 0.0;
    double y = 0.0;
    double totalAngle = 0.0;

    jointPositions.push_back(Matriks(vector<vector<double>>{
        {x},
        {y}
    }));

    for (int i = 0; i < lengths.size(); i++) {
        totalAngle += angles[i];
        x += cos(totalAngle) * lengths[i];
        y += sin(totalAngle) * lengths[i];

        jointPositions.push_back(Matriks(vector<vector<double>>{
            {x},
            {y}
        }));
    }

    return jointPositions;
}

// Compute alpha for jacobian transpose
double computeAlpha(Matriks J, Matriks errorPos) {
    Matriks eT = errorPos;
    eT.transpose();

    Matriks JT = J;
    JT.transpose();

    Matriks JJT = J * JT;
    Matriks JJTe = JJT * errorPos;

    double numerator = errorPos.getData()[0][0] * JJTe.getData()[0][0] + errorPos.getData()[1][0] * JJTe.getData()[1][0];
    double denominator = JJTe.getData()[0][0] * JJTe.getData()[0][0] + JJTe.getData()[1][0] * JJTe.getData()[1][0];

    if(abs(denominator) < 1e-12) {
        return 0.0;
    }

    return numerator / denominator;
}

// Inverse Kinematics Method : Jacobian transpose
vector<IKResult> jacobianTranspose(vector<double>& lengths, vector<double>& angles, Matriks target, vector<double>& minAngles, vector<double>& maxAngles) {
    // Check if data invalid or not
    if (lengths.size() != angles.size() || angles.size() != minAngles.size() || angles.size() != maxAngles.size()) {
        throw invalid_argument("[ERROR] Data invalid");
    }

    // Get the End effector position using inverse kinematics
    int jointCount = lengths.size();
    vector<Matriks> derivatives(jointCount);
    bool found = false;
    int iteration = 0;

    // Result data stored in vector of IKResult
    vector<IKResult> result;

    while(!found && iteration < MAX_ITERATION) {
        // Get the end effector position
        Matriks endEffector = forwardKinematics(lengths, angles); 

        // Compute the error from the end effector position to the target position
        Matriks errorPos = error(target, endEffector);

        // Insert the result for each iteratiom
        IKResult currentResult;
        currentResult.angles = angles;
        currentResult.endEffector = endEffector;
        currentResult.error = errorPos;

        result.push_back(currentResult);

        for(int i = 0; i < jointCount; i++) {
            Matriks endEffector = forwardKinematics(lengths, angles); 
            angles[i] += DEGREE;

            // Get the end-effector position after the small rotation
            Matriks test = forwardKinematics(lengths, angles); 
            angles[i] -= DEGREE;

            // Approximate the partial derivative of the end effector position
            // with respect to the current joint angle
            Matriks difference = error(test, endEffector);
            derivatives[i] = Matriks(vector<vector<double>>{
                {difference.getData()[0][0] / DEGREE},
                {difference.getData()[1][0] / DEGREE},
            });
        };
        
        // Create the Jacobian Matriks 
        // Using the Jacobian Transpose
        Matriks J = jacobianMatriks(derivatives);
        double a = computeAlpha(J, errorPos);
        J.transpose();

        // Compute the suggested angular change for each joint using J Transpose × error
        // for each joint for each iteration
        Matriks change = J * errorPos;

        // Update each angle using step size (a)
        for(int i = 0; i < jointCount; i++) {
            angles[i] += a * (change.getData()[i][0]);
            
            // Constraint the angles
            if (angles[i] < minAngles[i]){
                angles[i] = minAngles[i];
            }

            if (angles[i] > maxAngles[i]){
                angles[i] = maxAngles[i];
            }
        };

        double errorNorm = sqrt(pow(errorPos.getData()[0][0], 2) + pow(errorPos.getData()[1][0], 2));
        if(errorNorm < TOLERANCE) {
            found = true;
        }

        iteration++;
    }

    return result;
};

// Inverse Kinematics improvement : Damped Least Squares / LM 
vector<IKResult> DampedLeastSquares(vector<double>& lengths, vector<double>& angles, Matriks target, vector<double>& minAngles, vector<double>& maxAngles, vector<Obstacle> objects) {
    // Check if data invalid or not
    if (lengths.size() != angles.size() || angles.size() != minAngles.size() || angles.size() != maxAngles.size()) {
        throw invalid_argument("[ERROR] Data invalid");
    }

    // Get the End effector position using inverse kinematics
    int jointCount = lengths.size();
    vector<Matriks> derivatives(jointCount);
    int iteration = 0;
    bool found = false;

    // Result data stored in vector of IKResult
    vector<IKResult> result;

    while(!found && iteration < MAX_ITERATION) {
        // Get the end effector position
        Matriks endEffector = forwardKinematics(lengths, angles); 

        // Compute the error from the end effector position to the target position
        Matriks errorPos = error(target, endEffector);

        // Insert the result for each iteratiom
        IKResult currentResult;
        currentResult.angles = angles;
        currentResult.endEffector = endEffector;
        currentResult.error = errorPos;

        result.push_back(currentResult);

        for(int i = 0; i < jointCount; i++) {
            Matriks endEffector = forwardKinematics(lengths, angles); 
            angles[i] += DEGREE;

            // Get the end-effector position after the small rotation
            Matriks test = forwardKinematics(lengths, angles); 
            angles[i] -= DEGREE;

            // Approximate the partial derivative of the end effector position
            // with respect to the current joint angle
            Matriks difference = error(test, endEffector);
            derivatives[i] = Matriks(vector<vector<double>>{
                {difference.getData()[0][0] / DEGREE},
                {difference.getData()[1][0] / DEGREE},
            });
        };
        
        // Create the Jacobian Matriks 
        // Using Damped least Squares / Levenberg–Marquardt
        Matriks J = jacobianMatriks(derivatives);
        Matriks JT = J; JT.transpose();
        Matriks JJT = J * JT;

        // Check near singularity using the determinant of J * J transpose
        double determinant = JJT.getData()[0][0] * JJT.getData()[1][1] - JJT.getData()[1][0] * JJT.getData()[0][1];
        
        // Use adaptive lambda depends on the singularity
        double lambda;
        if (sqrt(abs(determinant)) < 100.0) {
            // Increase the lamda value if its near singularity 
            lambda = MAX_LAMBDA;
        } else {
            lambda = MIN_LAMBDA;
        }

        double lambda2 = pow(lambda, 2.0);
        Matriks lambda2I = Matriks::identity(JJT.getRow());

        for(int i = 0; i < lambda2I.getRow(); i++) {
            for(int j = 0; j < lambda2I.getColumn(); j++) {
                lambda2I.getData()[i][j] *= lambda2;
            }
        }

        Matriks A(JJT.getRow(), JJT.getColumn());
        for(int i = 0; i < JJT.getRow(); i++) {
            for(int j = 0; j < JJT.getColumn(); j++) {
                A.getData()[i][j] += (JJT.getData()[i][j] + lambda2I.getData()[i][j]);
            }
        }

        Matriks AInvers = A.inverse();
        Matriks AInversE = AInvers * errorPos;

        Matriks deltaTetha = JT * AInversE;
        for (int i = 0; i < jointCount; i++) {
            // Clamp delta tetha so it doesnt rotate extremely
            if (deltaTetha.getData()[i][0] > MAX_DELTA_TETHA) {
                deltaTetha.getData()[i][0] = MAX_DELTA_TETHA;
            }
            
            if (deltaTetha.getData()[i][0] < -MAX_DELTA_TETHA) {
                deltaTetha.getData()[i][0] = - MAX_DELTA_TETHA;
            }

            angles[i] += deltaTetha.getData()[i][0];

            // Constraint the angles
            if (angles[i] < minAngles[i]){
                angles[i] = minAngles[i];
            }

            if (angles[i] > maxAngles[i]){
                angles[i] = maxAngles[i];
            }
        }

        double errorNorm = sqrt(pow(errorPos.getData()[0][0], 2) + pow(errorPos.getData()[1][0], 2));
        if(errorNorm < TOLERANCE) {
            found = true;
        }

        iteration++;
    }    

    return result;
}
