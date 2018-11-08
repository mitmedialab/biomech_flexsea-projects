#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "linear_algebra_methods.h"
 
void cholesky_decomposition(float* A, float* LT, int n) {
 
    for (int i = 0; i < n; i++)
        for (int j = 0; j < (i+1); j++) {
            float s = 0;
            for (int k = 0; k < j; k++)
                s += LT[i * n + k] * LT[j * n + k];
            LT[i * n + j] = (i == j) ?
                           sqrt(A[i * n + i] - s) :
                           (1.0 / LT[j * n + j] * (A[i * n + j] - s));
        } 
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
        for (int j = i+1;j < n; j++)
        {
            R[j*n + i] = R[i*n + j];
            R[i*n + j] = 0.0;
        }
    }
}

void lower_to_upper_transpose(float *R, int n)
{

    for (int i = 0;i < n; i++)
    {
        for (int j = i+1;j < n; j++)
        {
            R[i*n + j] = R[j*n + i];
            R[j*n + i] = 0.0;
        }
    }
}



void backward_substitution(float *A, float *b, float* x, int n)
{

x[n-1] = b[n-1]/A[(n-1)*(n+1)];

 for (int i=n-2; i>=0; i--)
    {
      x[i] = b[i];        
        for (int j=i+1; j<n; j++)
        {
             x[i] -= A[(i)*n + j]*x[j];
        };             
      x[i] = x[i] / A[(i)*n + i];
    }; 
}

void forward_substitution(float *A, float *b, float* x, int n){
    
    x[0] = b[0]/A[0];

    for (int i = 1; i <= n-1; i++)
    {
        x[i] = b[i];
        for (int j = 0; j < i; j++)
        {
            x[i] -= A[i*n + j]*x[j];
        }

        x[i]= x[i] / A[(i)*n + i];
    }
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