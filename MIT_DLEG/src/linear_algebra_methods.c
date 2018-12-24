#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "linear_algebra_methods.h"
 
void cholesky(float* A, float* LT, int n) {
 
    for (int i = 0; i < n; i++){
        int i_x_n = i*n;

        for (int j = 0; j < (i+1); j++) {
            int j_x_n = j*n;
            float s = 0;
            for (int k = 0; k < j; k++)
                s += LT[i_x_n + k] * LT[j_x_n + k];
            LT[i_x_n + j] = (i == j) ?
                           sqrtf(A[i_x_n + i] - s) :
                           (1.0 / LT[j_x_n + j] * (A[i_x_n + j] - s));
        } 
    }
}

void segmented_cholesky(float* A, float* LT, int n, int i) {
 
    int i_x_n = i*n;
    int i_x_n_p_1 = i_x_n + i;
    for (int j = 0; j <= i; j++) {
        int j_x_n = j*n;
        float s = 0.0;
        for (int k = 0; k < j; k++)
            s += LT[i_x_n + k] * LT[j_x_n + k];
        LT[i_x_n + j] = (i == j) ?
                       sqrtf(A[i_x_n_p_1] - s) :
                       (1.0 / LT[j_x_n + j] * (A[i_x_n + j] - s));
    } 
}

void super_segmented_cholesky(float* A, float* LT, int n, int i, int j) {
 
    int i_x_n = i*n;
    int i_x_n_p_1 = i_x_n + i;
    int j_x_n = j*n;
    float s = inner_product(&LT[i_x_n],&LT[j_x_n],j);
    LT[i_x_n + j] = (i == j) ?
                   sqrtf(A[i_x_n_p_1] - s) :
                   (1.0 / LT[j_x_n + j] * (A[i_x_n + j] - s));
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
        int i_x_n = i*n;
        for (int j = i+1;j < n; j++)
        {
            R[j*n + i] = R[i_x_n + j];
            R[i_x_n + j] = 0.0;
        }
    }
}


void lower_to_upper_transpose(float *LT, float *UT, int n)
{

    for (int i = 0;i < n; i++)
    {
        int i_x_n = i*n;
        for (int j = i;j < n; j++)
        {
            UT[i_x_n + j] = LT[j*n + i];
        }
    }
}

void segmented_lower_to_upper_transpose(float *LT, float *UT, int n, int i)
{
        int i_x_n = i*n;
        int ind = i_x_n - n + i;
        int startval = i_x_n + i;
        int endval = i_x_n + n;
        for (int j = startval; j < endval; j++)
        {
            ind = ind+n;
            UT[j] = LT[ind];
        }
}



void backward_substitution(float *A, float *b, float* x, int n)
{

    x[n-1] = b[n-1]/A[(n-1)*(n+1)];

    for (int i=n-2; i>=0; i--)
    {
        int i_x_n = i*n;
        x[i] = b[i];        
        for (int j=i+1; j<n; j++)
        {
            x[i] -= A[i_x_n + j]*x[j];
        };             
        x[i] = x[i] / A[i_x_n + i];
    }; 
}

void segmented_backward_substitution(float *A, float *b, float* x, int n, int i)
{
    if (i == n-1){
        x[n-1] = b[n-1]/A[(n-1)*(n+1)];
        return;
    }

    int i_x_n = i*n;
    x[i] = b[i];        
    for (int j=i+1; j<n; j++)
    {
        x[i] -= A[i_x_n + j]*x[j];
    };             
    x[i] = x[i] / A[i_x_n + i];
}

void forward_substitution(float *A, float *b, float* x, int n){
    
    x[0] = b[0]/A[0];

    for (int i = 1; i < n; i++)
    {
        int i_x_n = i*n;
        x[i] = b[i];
        for (int j = 0; j < i; j++)
        {
            x[i] -= A[i_x_n + j]*x[j];
        }

        x[i]= x[i] / A[i_x_n + i];
    }
}

void segmented_forward_substitution(float *A, float *b, float* x, int n, int i){
    if (i == 0){
        x[0] = b[0]/A[0];
        return;
    }

    int i_x_n = i*n;
    x[i] = b[i];
    for (int j = 0; j < i; j++)
    {
        x[i] -= A[i_x_n + j]*x[j];
    }

    x[i]= x[i] / A[i_x_n + i];
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

void segmented_outer_product (float* x, float* y, float* P, int n, int i){
    int i_x_n = i*n;
    for (int j = 0; j < n; j++)
        P[i_x_n+j] = x[i]*y[j];
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