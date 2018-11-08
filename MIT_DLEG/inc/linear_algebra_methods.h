
#ifndef __LINEARALGEBRAMETHODS_H__
#define __LINEARALGEBRAMETHODS_H__
 

extern void cholesky_decomposition(float* A, float* LT, int n);
extern void backward_substitution (float *A, float *b, float *x, int n);
extern void forward_substitution (float *A, float *b, float *x, int n);
extern void transpose(float *A, float *B, int n);
extern void upper_to_lower_transpose(float *R, int n);
extern void lower_to_upper_transpose(float *R, int n);

extern float inner_product (float* x, float* y, int n);
extern void outer_product (float* x, float* y, float* P, int n);
extern void sum (float* x, float* y, float* z, int n);
extern void diff (float* x, float* y, float* z, int n);
extern void scaling (float* x, float a, float* z, int n);
extern void assignment (float* x, float* z, int n);

 #endif