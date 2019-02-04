
#ifndef __LINEARALGEBRAMETHODS_H__
#define __LINEARALGEBRAMETHODS_H__
 

extern void cholesky(float* A, float* LT, int n);
extern void segmented_cholesky(float* A, float* LT, int n, int current_feature);
extern void super_segmented_cholesky(float* A, float* LT, int n, int segment, int subsegment);
extern void backward_substitution (float *A, float *b, float *x, int n);
extern void segmented_backward_substitution(float *A, float *b, float* x, int n, int segment);
extern void forward_substitution (float *A, float *b, float *x, int n);
extern void segmented_forward_substitution(float *A, float *b, float* x, int n, int segment);
extern void transpose(float *A, float *B, int n);
extern void upper_to_lower_transpose(float *R, int n);
extern void lower_to_upper_transpose(float *LT, float *UT, int n);
extern void segmented_lower_to_upper_transpose(float *LT, float *UT, int n, int segment);

extern float inner_product (float* x, float* y, int n);
extern void outer_product (float* x, float* y, float* P, int n);
extern void segmented_outer_product (float* x, float* y, float* P, int n, int segment);
extern void sum (float* x, float* y, float* z, int n);
extern void diff (float* x, float* y, float* z, int n);
extern void scaling (float* x, float a, float* z, int n);
extern void assignment (float* x, float* z, int n);

 #endif