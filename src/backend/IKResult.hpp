#ifndef IKRESULT_HPP
#define IKRESULT_HPP

#include "iostream"
#include "vector"
#include "math/matrix.hpp"
using namespace std;

struct IKResult {
    vector<double> angles;
    Matrix endEffector;
    Matrix error;
};

#endif