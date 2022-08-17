#ifndef __MAGENTO_1_4__
#define __MAGENTO_1_4__

// In https://github.com/jremington/MPU-9250-AHRS
// magneto 1.4 magnetometer/accelerometer calibration code
// from http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// tested and works with Code::Blocks 10.05 through 20.03

// Command line version slightly modified from original, sjames_remington@gmail.com
// Now includes option to reject outliers in units of sigma, deviation of data vector length
// from mean values of the sample. Suggest using 2 as the rejection criterion

// comma separated ASCII input data file expected, three columns x, y, z

#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <float.h>

// Forward declarations of mymathlib.com routines
void Multiply_Self_Transpose(float*, float*, int, int);
void Get_Submatrix(float*, int, int, float*, int, int, int);
int Choleski_LU_Decomposition(float*, int);
int Choleski_LU_Inverse(float*, int);
void Multiply_Matrices(float*, float*, int, int, float*, int);
void Identity_Matrix(float*, int);

int Hessenberg_Form_Elementary(float*, float*, int);
void Hessenberg_Elementary_Transform(float*, float*, int[], int);

void Copy_Vector(float*, float*, int);

int QR_Hessenberg_Matrix(float*, float*, float[], float[], int, int);
void One_Real_Eigenvalue(float[], float[], float[], int, float);
void Two_Eigenvalues(float*, float*, float[], float[], int, int, float);
void Update_Row(float*, float, float, int, int);
void Update_Column(float*, float, float, int, int);
void Update_Transformation(float*, float, float, int, int);
void Double_QR_Iteration(float*, float*, int, int, int, float*, int);
void Product_and_Sum_of_Shifts(float*, int, int, float*, float*, float*, int);
int Two_Consecutive_Small_Subdiagonal(float*, int, int, int, float, float);
void Double_QR_Step(float*, int, int, int, float, float, float*, int);
void BackSubstitution(float*, float[], float[], int);
void BackSubstitute_Real_Vector(float*, float[], float[], int, float, int);
void BackSubstitute_Complex_Vector(float*, float[], float[], int, float, int);
void Calculate_Eigenvectors(float*, float*, float[], float[], int);
void Complex_Division(float, float, float, float, float*, float*);

void Transpose_Square_Matrix(float*, int);

int Lower_Triangular_Solve(float* L, float B[], float x[], int n);
int Lower_Triangular_Inverse(float* L, int n);
int Upper_Triangular_Solve(float* U, float B[], float x[], int n);

void Interchange_Rows(float* A, int row1, int row2, int ncols);
void Interchange_Columns(float* A, int col1, int col2, int nrows, int ncols);
void Identity_Matrix(float* A, int n);
void Copy_Vector(float* d, float* s, int n);

void Hessenberg_Elementary_Transform(float* H, float* S, int perm[], int n);

void One_Real_Eigenvalue(float Hrow[], float eigen_real[], float eigen_imag[], int row, float shift);
void Two_Eigenvalues(float* H, float* S, float eigen_real[], float eigen_imag[], int n, int k, float t);
void Update_Row(float* Hrow, float cos, float sin, int n, int k);
void Update_Column(float* H, float cos, float sin, int n, int k);
void Update_Transformation(float* S, float cos, float sin, int n, int k);
void Double_QR_Iteration(float* H, float* S, int row, int min_row, int n, float* shift, int iteration);
void Product_and_Sum_of_Shifts(float* H, int n, int max_row, float* shift, float* trace, float* det, int iteration);
int Two_Consecutive_Small_Subdiagonal(float* H, int min_row, int max_row, int n, float trace, float det);
void Double_QR_Step(float* H, int min_row, int max_row, int min_col, float trace, float det, float* S, int n);
void Complex_Division(float x, float y, float u, float v, float* a, float* b);
void BackSubstitution(float* H, float eigen_real[], float eigen_imag[], int n);
void BackSubstitute_Real_Vector(float* H, float eigen_real[], float eigen_imag[], int row, float zero_tolerance, int n);
void BackSubstitute_Complex_Vector(float* H, float eigen_real[], float eigen_imag[], int row, float zero_tolerance, int n);
void Calculate_Eigenvectors(float* H, float* S, float eigen_real[], float eigen_imag[], int n);

void CalculateCalibration(float *buf, int sampleCount, float BAinv[4][3]);

#endif // __MAGENTO_1_4__