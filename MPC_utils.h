#pragma once
#include "mkl.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

int CalculateInverseMatrix(double* pDst, const double* pSrc, int dim);
inline void swap(double** a, double** b)
{
	double* tmp = *b;
	*b = *a;
	*a = tmp;
}

bool GenerateSu(double* Su, const double* A, const double* B, const double* C, int n, int p, int m)
{
	double* last_power = (double*)mkl_calloc(n * n , sizeof(double), 64);
	double* power = (double*)mkl_calloc(n * n , sizeof(double), 64);
	double* middle = (double*)mkl_calloc(n * 1 , sizeof(double), 64);
	
	if (Su == NULL || A == NULL || B == NULL || C == NULL || last_power == NULL || power == NULL || middle == NULL)
		return false;

	for (int row = 0; row < p; row++)
		for (int col = 0; col < m; col++)
		{
			for (int i = 0; i < n * n; i++)
			{
				last_power[i] = 0;
				power[i] = 0;
			}
			for (int i = 0; i < n; i++)
			{
				last_power[i * n + i] = 1;
				power[i * n + i] = 1;
			}
			for (int k = 0; k <= row - col; k++)
			{
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
					n, 1, n, 1.0, last_power, n, B, 1, 0, middle, 1);
				double res = 0;
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
					1, 1, n, 1.0, C, n, middle, 1, 0, &res, 1);
				Su[row * m + col] += res;
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
					n, n, n, 1.0, A, n, last_power, n, 0, power, n);
				swap(&last_power, &power);
				//swap the pointers to avoid unnecessary memory copy
			}
		}
	mkl_free(last_power);
	mkl_free(power);
	mkl_free(middle);
	return true;
}


bool GenerateWeightMatrix(double* W, int size, double weight)
{
	if (W == NULL) return false;
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			W[i * size + j] = i == j ? weight * weight : 0;
	return true;
}

bool GenerateExponentialWeightMatrix(double* W, int size, double start_weight, double end_weight)
{
	if (size == 1)
	{
		if (W == NULL) return false;
		W[0] = start_weight;
	}
	else
	{
		double multiplier = pow(end_weight / start_weight, 2.0 / (size - 1));
		double tmp = start_weight * start_weight;
		if (W == NULL) return false;
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
				if (i==j)
				{
					W[i * size + j] = tmp;
					tmp *= multiplier;
				}
				else
					W[i * size + j] = 0;
	}
	return true;
}

bool GenerateKmpc(double* Kmpc, const double* Su, const double* Wy, const double* Wu, int p, int m)
{
	if (Kmpc == NULL || Su == NULL || Wy == NULL || Wu == NULL)
		return false;
	double* middle = (double*)mkl_calloc(m * p , sizeof(double), 64);
	cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans,
		m, p, p, 1.0, Su, m, Wy, p, 0, middle, p);

	double* SWS_W = (double*)mkl_calloc(m * m , sizeof(double), 64);
	memcpy(SWS_W, Wu, m * m * sizeof(double));
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		m, m, p, 1.0, middle, p, Su, m, 1.0, SWS_W, m);

	int* inv_ax = (int*)mkl_calloc(m, sizeof(int), 32);

	LAPACKE_dgetrf(LAPACK_ROW_MAJOR, m, m, SWS_W, m, inv_ax);
	LAPACKE_dgetri(LAPACK_ROW_MAJOR, m, SWS_W, m, inv_ax);
    
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		m, p, m, 1.0, SWS_W, m, middle, p, 0, Kmpc, p);

    mkl_free(middle);
	mkl_free(SWS_W);
	mkl_free(inv_ax);
	return true;
}

bool GenerateSx(double* Sx,const double* A,const double* C, int n, int p)
{
	if (Sx == NULL || A == NULL || C == NULL)
		return false;
	double* C_A_last = (double*)mkl_calloc(1 * n , sizeof(double), 64);
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 1, n, n, 1.0, C, n, A, n, 0.0, C_A_last, n);
	double* lastrow_Sx = (double*)mkl_calloc(1 * n , sizeof(double), 64);
	memcpy(Sx, C_A_last, 1 * n * sizeof(double));
	memcpy(lastrow_Sx, Sx, 1 * n * sizeof(double));


	double* C_A_i = (double*)mkl_calloc(1 * n , sizeof(double), 64);

	for (int i = 1; i < p; i++)
	{
		cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 1, n, n, 1.0, C_A_last, n, A, n, 0.0, C_A_i, n);
		//C_A_i=C_A_last*A
		for (int j = 0; j < n; j++)
		{
			Sx[i * n + j] = lastrow_Sx[j] + C_A_i[j];
			//Sx(i,:)=Sx(i-1,:)+C_A_i;
		}
		memcpy(lastrow_Sx, Sx + i * n, 1 * n * sizeof(double));
		//update lastrow
		swap(&C_A_last, &C_A_i);
	}

	mkl_free(C_A_i);
	mkl_free(C_A_last);
	mkl_free(lastrow_Sx);
	return true;
}
