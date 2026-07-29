#include "iostream"
#include "vector"
#include "cmath"
using namespace std;

class Matrix {
private:
    vector<vector<double>> matrix;

public:
    Matrix() {
        this->matrix = {
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}
        };
    };

    Matrix(vector<vector<double>> value) {
        this->matrix = value;
    };

    vector<vector<double>> getValue() {
        return this->matrix;
    }

    Matrix operator*(const Matrix& other) const {
        Matrix result;

        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                for (int k = 0; k < 3; k++) {
                    result.matrix[row][column] += matrix[row][k] * other.matrix[k][column];
                }
            }
        }

        return result;
    }

    static Matrix rotation(double angle) {
        return Matrix({
            {cos(angle), -sin(angle), 0},
            {sin(angle), cos(angle), 0},
            {0, 0, 1}
        });
    };

    static Matrix translation(double x, double y) {
        return Matrix({
            {1, 0, x},
            {0, 1, y},
            {0, 0, 1}
        });
    };

    static Matrix homogeniousMatrix(double length, double angle) {
        return translation(length, 0.0) * rotation(angle);
    }
};
