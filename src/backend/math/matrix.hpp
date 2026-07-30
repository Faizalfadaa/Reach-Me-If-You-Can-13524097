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

        static Matrix identity() {
            return Matrix({
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
            });
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
