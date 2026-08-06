#ifndef MATRIKS_HPP
#define MATRIKS_HPP

#include "iostream"
#include "vector"
#include "cmath"
#include "stdexcept"
using namespace std;

class Matriks {
    private:
        vector<vector<double>> data;
        int row;
        int col;

    public:
        Matriks(){};
        
        Matriks(int row, int col) {
            this->data = vector<vector<double>>(row, vector<double>(col, 0.0));
            this->row = row;
            this->col = col;
        };

        Matriks(const vector<vector<double>>& data) {
            this->data = data;
            this->row = data.size();
            this->col = data[0].size();
        };

        vector<vector<double>>& getData() {
            return this->data;
        }

        int getColumn() const {
            return this->col;
        }

        int getRow() const {
            return this->row;
        }

        Matriks operator*(const Matriks& other) const {
            int leftRows = this->row;
            int leftColumns = this->col;
            int rightRows = other.getRow();
            int rightColumns = other.getColumn();

            if (leftColumns != rightRows) {
                throw invalid_argument("[ERROR] Dimensions are incompatible for multiplication");
            }

            Matriks result(leftRows, rightColumns);

            for (int row = 0; row < leftRows; row++) {
                for (int column = 0; column < rightColumns; column++) {
                    for (int k = 0; k < leftColumns; k++) {
                        result.data[row][column] += this->data[row][k] * other.data[k][column];
                    }
                }
            }

            return result;
        }

        Matriks inverse() const {
            if (this->row != this->col) {
                throw invalid_argument("[ERROR] Matriks is not square");
            }

            int n = this->row;
            Matriks augmented(n, 2 * n);

            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    augmented.data[i][j] = this->data[i][j];
                }

                augmented.data[i][n + i] = 1.0;
            }

            // Proses eliminasi Gauss Jordan
            for (int i = 0; i < n; i++) {
                int pivotRow = i;
                for (int j = i + 1; j < n; j++) {
                    if (fabs(augmented.data[j][i]) > fabs(augmented.data[pivotRow][i])) {
                        pivotRow = j;
                    }
                }

                if (fabs(augmented.data[pivotRow][i]) < 1e-12) {
                    throw invalid_argument("[ERROR] Matriks is singular");
                }

                if (pivotRow != i) {
                    vector<double> temp = augmented.data[i];
                    augmented.data[i] = augmented.data[pivotRow];
                    augmented.data[pivotRow] = temp;
                }

                double pivot = augmented.data[i][i];
                for (int j = 0; j < 2 * n; j++) {
                    augmented.data[i][j] /= pivot;
                }

                for (int j = 0; j < n; j++) {
                    if (j != i) {
                        double factor = augmented.data[j][i];
                        for (int k = 0; k < 2 * n; k++) {
                            augmented.data[j][k] -= factor * augmented.data[i][k];
                        }
                    }
                }
            }

            Matriks result(n, n);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    result.data[i][j] = augmented.data[i][n + j];
                }
            }

            return result;
        };

        void transpose() {
            vector<vector<double>> result(this->col, vector<double>(this->row));

            for (int row = 0; row < this->row; row++) {
                for (int column = 0; column < this->col; column++) {
                    result[column][row] = this->data[row][column];
                }
            }

            this->data = result;
            int temp = this->row;
            this->row = this->col;
            this->col = temp;
        };

        static double distance(Matriks m1, Matriks m2) {
            return sqrt(pow(m1.data[0][0] - m2.data[0][0], 2) + pow(m1.data[1][0] - m2.data[1][0], 2));
        }

        static double coordinateX(Matriks& point) {
            return point.data[0][0];
        }

        static double coordinateY(Matriks& point) {
            return point.data[1][0];
        }

        static Matriks point2D(double x, double y) {
            return Matriks(vector<vector<double>>{
                {x}, 
                {y}
            });
        }

        static double dotProduct(Matriks& a, Matriks& b) {
            return coordinateX(a) * coordinateX(b) + coordinateY(a) * coordinateY(b);
        }

        static Matriks normalize(Matriks vector) {
            double length = sqrt(dotProduct(vector, vector));

            if (length < 1e-12) {
                return point2D(0.0, 0.0);
            }

            return point2D(coordinateX(vector) / length, coordinateY(vector) / length);
        }

        static Matriks identity(int n) {
            vector<vector<double>> I;
            for(int i = 0; i < n; i++) {
                vector<double> row = {};
                for(int j = 0; j < n; j++) {
                    if(i == j) {
                        row.push_back(1.0);
                    } else {
                        row.push_back(0.0);
                    }
                }
                I.push_back(row);
            }
            return Matriks(I);
        }

        static Matriks rotation(double angle) {
            return Matriks({
                {cos(angle), -sin(angle), 0},
                {sin(angle), cos(angle), 0},
                {0, 0, 1}
            });
        };

        static Matriks translation(double x, double y) {
            return Matriks({
                {1, 0, x},
                {0, 1, y},
                {0, 0, 1}
            });
        };

        static Matriks homogeniousMatriks(double length, double angle) {
            return rotation(angle) * translation(length, 0.0);
        }
};

#endif
