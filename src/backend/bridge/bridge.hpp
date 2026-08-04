#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include "../math/matrix.hpp"
#include "../../frontend/circle.cpp"
#include "vector"

#define PI 3.14159265358979323846f
#define DEG2RAD (PI/180.0f) 

using namespace std;

struct Obstacle {
    Matriks midPoint;
    double radius;
};

// Convert a GUI screen point into backend math coordinates
Matriks convertTarget(float targetX, float targetY, float baseX, float baseY) {
    return Matriks(vector<vector<double>>{
        {targetX - baseX},
        {baseY - targetY}
    });
}

// Convert a backend math point back into GUI coordinates
vector<double> convertToScreenPosition(Matriks point, float baseX, float baseY) {
    return {
        (float)(baseX + point.getData()[0][0]),
        (float)(baseY - point.getData()[1][0])
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

// Covert Circle data into Obstacle for backend
vector<Obstacle> convertCircles(vector<Circle>& obstacles, float baseX, float baseY) {
    vector<Obstacle> objects;

    for(int i = 0; i < obstacles.size(); i++) {
        Obstacle object;
        object.midPoint = Matriks(vector<vector<double>>{
            {obstacles[i].center.x - baseX},
            {baseY - obstacles[i].center.y}
        });
        object.radius = obstacles[i].radius;

        objects.push_back(object);
    }

    return objects;
}

#endif
