#include "iostream"
#include "vector"
#include "cmath"
#include "stdexcept"
using namespace std;

class Matrix {
    private:
        vector<vector<double>> data;
        int row;
        int col;

    public:
        Matrix(){};
        
        Matrix(int row, int col) {
            this->data = vector<vector<double>>(row, vector<double>(col, 0.0));
            this->row = row;
            this->col = col;
        };

        Matrix(const vector<vector<double>>& data) {
            this->data = data;
            this->row = data.size();
            this->col = data[0].size();
        };

        vector<vector<double>> getData() const {
            return this->data;
        }

        int getColumn() const {
            return this->col;
        }

        int getRow() const {
            return this->row;
        }

        Matrix operator*(const Matrix& other) const {
            int leftRows = this->row;
            int leftColumns = this->col;
            int rightRows = other.getRow();
            int rightColumns = other.getColumn();

            if (leftColumns != rightRows) {
                throw invalid_argument("[ERROR] Dimensions are incompatible for multiplication");
            }

            Matrix result(leftRows, rightColumns);

            for (int row = 0; row < leftRows; row++) {
                for (int column = 0; column < rightColumns; column++) {
                    for (int k = 0; k < leftColumns; k++) {
                        result.data[row][column] += this->data[row][k] * other.data[k][column];
                    }
                }
            }

            return result;
        }

        Matrix inverse() const {
            if (this->row != this->col) {
                throw invalid_argument(
                    "[ERROR] Matrix is not square"
                );
            }

            int n = this->row;
            Matrix augmented(n, 2 * n);

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
                    if ((augmented.data[j][i]) > (augmented.data[pivotRow][i])) {
                        pivotRow = j;
                    }
                }

                if (fabs(augmented.data[pivotRow][i]) < 1e-12) {
                    throw invalid_argument("[ERROR] Matrix is singular");
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
                            augmented.data[j][k] -=
                                factor * augmented.data[i][k];
                        }
                    }
                }
            }

            Matrix result(n, n);
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

        static Matrix identity(int n) {
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
            return Matrix(I);
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
            return rotation(angle) * translation(length, 0.0);
        }
};
