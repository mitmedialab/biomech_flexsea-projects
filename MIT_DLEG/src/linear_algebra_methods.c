#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "linear_algebra_methods.h"
 
void cholesky_decomposition(float* A, float* LT, int n) {
 
    for (int i = 0; i < n; i++){
        int i_times_n = i*n;

        for (int j = 0; j < (i+1); j++) {
            int j_times_n = j*n;
            float s = 0;
            for (int k = 0; k < j; k++)
                s += LT[i_times_n + k] * LT[j_times_n + k];
            LT[i_times_n + j] = (i == j) ?
                           sqrt(A[i_times_n + i] - s) :
                           (1.0 / LT[j_times_n + j] * (A[i_times_n + j] - s));
        } 
    }
}

void segmented_cholesky_decomposition(float* A, float* LT, int n, int segment) {
 
    int segment_times_n = segment*n;
    int segment_times_n_p_1 = segment_times_n + segment;
    for (int j = 0; j <= segment; j++) {
        int j_times_n = j*n;
        float s = 0.0;
        for (int k = 0; k < j; k++)
            s += LT[segment_times_n + k] * LT[j_times_n + k];
        LT[segment_times_n + j] = (segment == j) ?
                       sqrt(A[segment_times_n_p_1] - s) :
                       (1.0 / LT[j_times_n + j] * (A[segment_times_n + j] - s));
    } 
}

void super_segmented_cholesky_decomposition_v2(float* A, float* LT, int n, int segment, int subsegment) {
 
    int segment_times_n = segment*n;
    int segment_times_n_p_1 = segment_times_n + segment;
    int j = subsegment;
    int j_times_n = j*n;
    float s = 0.0;
    for (int k = 0; k < j; k++)
        s += LT[segment_times_n + k] * LT[j_times_n + k];
    LT[segment_times_n + j] = (segment == j) ?
                   sqrt(A[segment_times_n_p_1] - s) :
                   (1.0 / LT[j_times_n + j] * (A[segment_times_n + j] - s));
}

void super_segmented_cholesky_decomposition(float* A, float* LT, int n, int segment1, int segment2) {
 
    int segment1_times_n = segment1*n;
    int segment1_times_n_p_1 = segment1_times_n + segment1;

        int j = segment2;
        int j_times_n = j*n;
        float s = 0.0;
        for (int k = 0; k < j; k++)
            s += LT[segment1_times_n + k] * LT[j_times_n + k];
        LT[segment1_times_n + j] = (segment1 == j) ?
                       sqrt(A[segment1_times_n_p_1] - s) :
                       (1.0 / LT[j_times_n + j] * (A[segment1_times_n + j] - s));

        j = segment1-segment2;
        if (j == segment2)
            return;
        j_times_n = j*n;
        s = 0.0;
        for (int k = 0; k < j; k++)
            s += LT[segment1_times_n + k] * LT[j_times_n + k];
        LT[segment1_times_n + j] = (segment1 == j) ?
                       sqrt(A[segment1_times_n_p_1] - s) :
                       (1.0 / LT[j_times_n + j] * (A[segment1_times_n + j] - s));
}
 
void transpose(float *A, float* B, int n)
{

    for (int i = 0;i < n; i++)
    {
        for (int j = 0;j < n; j++)
        {
            B[i*n + j] = A[j*n + i];
        }
    }
}

void upper_to_lower_transpose(float *R, int n)
{

    for (int i = 0;i < n; i++)
    {
        int i_times_n = i*n;
        for (int j = i+1;j < n; j++)
        {
            R[j*n + i] = R[i_times_n + j];
            R[i_times_n + j] = 0.0;
        }
    }
}


void lower_to_upper_transpose(float *LT, float *UT, int n)
{

    for (int i = 0;i < n; i++)
    {
        int i_times_n = i*n;
        for (int j = i;j < n; j++)
        {
            UT[i_times_n + j] = LT[j*n + i];
        }
    }
}

void segmented_lower_to_upper_transpose(float *LT, float *UT, int n, int segment)
{
        int segment_times_n = segment*n;
        for (int j = segment;j < n; j++)
        {
            UT[segment_times_n + j] = LT[j*n + segment];
        }
}



void backward_substitution(float *A, float *b, float* x, int n)
{

    x[n-1] = b[n-1]/A[(n-1)*(n+1)];

    for (int i=n-2; i>=0; i--)
    {
        int i_times_n = i*n;
        x[i] = b[i];        
        for (int j=i+1; j<n; j++)
        {
            x[i] -= A[i_times_n + j]*x[j];
        };             
        x[i] = x[i] / A[i_times_n + i];
    }; 
}

void segmented_backward_substitution(float *A, float *b, float* x, int n, int segment)
{
    if (segment == n-2)
        x[n-1] = b[n-1]/A[(n-1)*(n+1)];

    int segment_times_n = segment*n;
    x[segment] = b[segment];        
    for (int j=segment+1; j<n; j++)
    {
        x[segment] -= A[segment_times_n + j]*x[j];
    };             
    x[segment] = x[segment] / A[segment_times_n + segment];
}

void forward_substitution(float *A, float *b, float* x, int n){
    
    x[0] = b[0]/A[0];

    for (int i = 1; i < n; i++)
    {
        int i_times_n = i*n;
        x[i] = b[i];
        for (int j = 0; j < i; j++)
        {
            x[i] -= A[i_times_n + j]*x[j];
        }

        x[i]= x[i] / A[i_times_n + i];
    }
}

void segmented_forward_substitution(float *A, float *b, float* x, int n, int segment){
    if (segment == 1)
        x[0] = b[0]/A[0];

    int segment_times_n = segment*n;
    x[segment] = b[segment];
    for (int j = 0; j < segment; j++)
    {
        x[segment] -= A[segment_times_n + j]*x[j];
    }

    x[segment]= x[segment] / A[segment_times_n + segment];
}

float inner_product (float* x, float* y, int n){
    float dp = 0.0;
    for (int i = 0; i < n; i++)
        dp = dp + x[i]*y[i];
    return dp;

}

void outer_product (float* x, float* y, float* P, int n){
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            P[i*n+j] = x[i]*y[j];
}

void segmented_outer_product (float* x, float* y, float* P, int n, int segment){
    int segment_times_n = segment*n;
    for (int j = 0; j < n; j++)
        P[segment_times_n+j] = x[segment]*y[j];
}

void sum (float* x, float* y, float* z, int n){
    for (int i = 0; i < n; i++)
        z[i] = x[i] + y[i];
}

void diff (float* x, float* y, float* z, int n){
    for (int i = 0; i < n; i++)
        z[i] = x[i] - y[i];
}

void scaling (float* x, float a, float* z, int n){
    for (int i = 0; i < n; i++)
        z[i] = x[i] * a;
}

void assignment(float* x, float* z, int n){
    for (int i = 0; i < n; i++)
        z[i] = x[i];
}