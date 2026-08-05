#ifndef IKRESULT_HPP
#define IKRESULT_HPP

#include "iostream"
#include "vector"
#include "math/matrix.hpp"
using namespace std;

enum IKStatus {
    IK_RUNNING,
    IK_REACHED,
    IK_UNREACHABLE_DISTANCE,
    IK_NOT_REACHED
};

struct IKResult {
    vector<double> angles;
    Matriks endEffector;
    Matriks error;
    IKStatus status;
};

#endif