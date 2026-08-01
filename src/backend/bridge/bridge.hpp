#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include "raylib.h"
#include "math/matrix.hpp"
#include "vector"

using namespace std;

// Convert a GUI screen point into backend math coordinates
Matrix convertTarget(Vector2 target, Vector2 base) {
    return Matrix(vector<vector<double>>{
        {target.x - base.x},
        {base.y - target.y}
    });
}

// Convert a backend math point back into GUI coordinates
Vector2 convertToScreenPosition(Matrix point, Vector2 base) {
    return Vector2{
        (float)(base.x + point.getData()[0][0]),
        (float)(base.y - point.getData()[1][0])
    };
}

// Convert length data from GUI to backend
vector<double> convertLengths(int lengths[5], int jointCount) {
    vector<double> newLengths;

    for(int i = 0; i < jointCount; i++) {
        newLengths.push_back(lengths[i]);
    }

    return newLengths;
}

// Convert engles data from GUI to backend
vector<double> convertAngles(float angles[5], int jointCount) {
    vector<double> newAngles;

    for(int i = 0; i < jointCount; i++) {
        newAngles.push_back(angles[i] * DEG2RAD);
    }

    return newAngles;
}

// Convert min angles data from GUI to backend
vector<double> convertMinAngles(int minAngles[5], int jointCount) {
    vector<double> newMinAngles;

    for(int i = 0; i < jointCount; i++) {
        newMinAngles.push_back(minAngles[i] * DEG2RAD);
    }

    return newMinAngles;
}

// Convert min angles data from GUI to backend
vector<double> convertMaxAngles(int maxAngles[5], int jointCount) {
    vector<double> newMaxAngles;

    for(int i = 0; i < jointCount; i++) {
        newMaxAngles.push_back(maxAngles[i] * DEG2RAD);
    }

    return newMaxAngles;
}

#endif
