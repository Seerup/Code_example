#ifndef LINEARALGEBRA_H_

#include <array>
#include <vector>
#include <iostream>
#include <stdlib.h>				// Use srand and rand

using namespace std;

class LinearAlgebra
{
    public:
        LinearAlgebra();

        void matrixMultiplication(double matrixA[12][6], double matrixB[12][6], double result[12][6]);
        void matrixMultiplication(double matrixA[3][3], double matrixB[3][2], double result[3][2]);
        void matrixMultiplication(double matrixA[1][12], double matrixB[12][6], double result[1][6]);
        vector<vector<double>> matrixMultiplication(vector<vector<double>> matrixA, vector<vector<double>> matrixB);

        void printMatrix(double matrix[3][2]);
        void printMatrix(double matrix[3][3]);
        void printMatrix(vector<vector<double>> aVector);
};



#endif