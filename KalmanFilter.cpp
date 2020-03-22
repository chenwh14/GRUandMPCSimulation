#include "KalmanFilter.h"
#include "mkl.h"
#include <string.h>
#include <stdio.h>
#include "simstruc.h"

void PrintMatrix(double* mat, int rows, int cols)
{
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			ssPrintf("%.16lf  ", mat[j + i * cols]);
		}
		ssPrintf("\n");
	}
	ssPrintf("\n\n");
}

KalmanFilter::KalmanFilter(const double* _A, const double* _B, const double* _C, int _n, const double* _P, const double* _Q, double _r) :n(_n), r(_r)
{
	A = (double*)mkl_calloc(n * n, sizeof(double), 64);
	B = (double*)mkl_calloc(n * 1, sizeof(double), 64);
	C = (double*)mkl_calloc(1 * n, sizeof(double), 64);
	memcpy(A, _A, n * n * sizeof(double));
	memcpy(B, _B, n * 1 * sizeof(double));
	memcpy(C, _C, 1 * n * sizeof(double));

	P = (double*)mkl_calloc(n * n, sizeof(double), 64);
	Q = (double*)mkl_calloc(n * n, sizeof(double), 64);
	memcpy(P, _P, n * n * sizeof(double));
	memcpy(Q, _Q, n * n * sizeof(double));

	K = (double*)mkl_calloc(n * 1, sizeof(double), 64);

	x_store = (double*)mkl_calloc(n * 1, sizeof(double), 64);
	x_based_on_last = (double*)mkl_calloc(n * 1, sizeof(double), 64);
	P_AT = (double*)mkl_calloc(n * n, sizeof(double), 64);
	P_CT = (double*)mkl_calloc(n * 1, sizeof(double), 64);
	C_P_CT_R = (double*)mkl_calloc(1 * 1, sizeof(double), 64);
}

KalmanFilter::~KalmanFilter()
{
	mkl_free(A);
	mkl_free(B);
	mkl_free(C);
	mkl_free(P);
	mkl_free(Q);
	mkl_free(K);
	mkl_free(x_store);
	mkl_free(x_based_on_last);
	mkl_free(P_AT);
	mkl_free(P_CT);
	mkl_free(C_P_CT_R);
}

void KalmanFilter::Update(double* x, double measured, double u_last)
{
	memcpy(x_based_on_last, B, n * sizeof(double));
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		n, 1, n, 1.0, A, n, x_store, 1, u_last, x_based_on_last, 1);
	//x_based_on_last=A*x_last+B*u_last;

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		n, n, n, 1.0, P, n, A, n, 0.0, P_AT, n);
	//calculate P*A��

	memcpy(P, Q, n * n * sizeof(double));
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		n, n, n, 1.0, A, n, P_AT, n, 1.0, P, n);
	//P=A*P*A'+Q;

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		n, 1, n, 1.0, P, n, C, n, 0.0, P_CT, 1);
	//calculate P*C'

	*C_P_CT_R = r;
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		1, 1, n, 1.0, C, n, P_CT, 1, 1.0, C_P_CT_R, 1);
	//calculate C*P*C'+R
	*C_P_CT_R = 1.0 / (*C_P_CT_R);

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		n, 1, 1, 1.0, P_CT, 1, C_P_CT_R, 1, 0.0, K, 1);
	//K=P*C'*(C*P*C'+R)^-1;
    
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		1, 1, n, -1.0, C, n, x_based_on_last, 1, 1.0, &measured, 1);
	//calculate measured-C*x_based_on_last
	//then store the result in measured

	memcpy(x_store, x_based_on_last, n * sizeof(double));
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		n, 1, 1, 1.0, K, 1, &measured, 1, 1.0, x_store, 1);
	//x_new=x_based_on_last+K*(measured-C*x_based_on_last);
	//where we directly let x_store be x_new

	memcpy(x, x_store, n * sizeof(double));
	//output
}