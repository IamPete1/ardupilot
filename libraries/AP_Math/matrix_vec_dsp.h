#pragma once

#include <AP_HAL/AP_HAL.h>

// Do not use DSP, it is much faster in some cases, but upto 2x slower in others
// Not using it gives more consitant run-times
#define use_DSP HAL_WITH_DSP && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && 0

#if use_DSP
#include <arm_math.h>
typedef arm_matrix_instance_f32 Matrix;
#else
struct Matrix {
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
};
#endif

struct MatrixD {
    uint16_t numRows;
    uint16_t numCols;
    double *pData;
};

// setup matrix struct
void init_mat(Matrix &S, uint16_t numRows, uint16_t numCols, float *pData);
void init_mat(MatrixD &S, uint16_t numRows, uint16_t numCols, double *pData);

// element wise matrix vector multiplication
void per_element_mult_mv(const Matrix &A, float *B, Matrix &dest);
void per_element_mult_mv(const MatrixD &A, double *B, MatrixD &dest);

// transpose
void mat_trans(const Matrix &A, Matrix &dest);
void mat_trans(const MatrixD &A, MatrixD &dest);

// multiply two matrices
void mat_mult(const Matrix &A, const Matrix &B, Matrix &dest);
void mat_mult(const MatrixD &A, const MatrixD &B, MatrixD &dest);

// scale by constant
void vec_scale(const float *A, const float scale, float *dest, uint8_t size);
void vec_scale(const double *A, const double scale, double *dest, uint8_t size);

// set all values
void vec_fill(const float value, float *dest, uint8_t size);
void vec_fill(const double value, double *dest, uint8_t size);

// element wise add
void vec_add(const float *A, const float *B, float *dest, uint8_t size);
void vec_add(const double *A, const double *B, double *dest, uint8_t size);

// element wise subtract
void vec_sub(const float *A, const float *B, float *dest, uint8_t size);
void vec_sub(const double *A, const double *B, double *dest, uint8_t size);

// element wise multiply
void vec_mult(const float *A, const float *B, float *dest, uint8_t size);
void vec_mult(const double *A, const double *B, double *dest, uint8_t size);

// dot product
void dot_prod(const float *A, const float *B, uint8_t size, float *dest);
void dot_prod(const double *A, const double *B, uint8_t size, double *dest);

// add constant
void vec_offset(const float *A, const float offset, float *dest, uint8_t size);
void vec_offset(const double *A, const double offset, double *dest, uint8_t size);

// element wise invert
void vec_inv(const float *A, float *dest, uint8_t size);
void vec_inv(const double *A, double *dest, uint8_t size);

// matrix multiplied by vector
void mat_vec_mult(const Matrix &A, const float *B, float *dest);
void mat_vec_mult(const MatrixD &A, const double *B, double *dest);

// in place Cholesky factorisation
bool cholesky(Matrix &A);
bool cholesky(MatrixD &A);

// forward substitution
void forward_sub(const Matrix &A, const float *B, float *dest);
void forward_sub(const MatrixD &A, const double *B, double *dest);

// backwards substitution, transposing a
void backward_sub_t(const Matrix &A, const float *B, float *dest);
void backward_sub_t(const MatrixD &A, const double *B, double *dest);

// print matrix to console, for debugging (SITL only)
void print_mat(const char* name, const Matrix &A);
void print_mat(const char* name, const MatrixD &A);
