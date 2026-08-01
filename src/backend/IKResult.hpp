#ifndef IKRESULT_HPP
#define IKRESULT_HPP

#include "iostream"
#include "vector"
#include "math/matrix.hpp"
using namespace std;

struct IKResult {
    vector<double> angles;
    vector<vector<double>> jointPositions;
    Matrix endEffector;
    Matrix error;
};

#endif