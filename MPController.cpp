#include "MPController.h"
#include "MKL.h"
#include <string.h>
#include "MPC_utils.h"
#include "simstruc.h"

MPController::MPController(const double* _A, const double* _B, const double* _C, int _n, int _p, int _m, double _wy, double _wu):n(_n),p(_p),m(_m),mv_store(0)
{
	A = (double*)mkl_calloc(n * n , sizeof(double), 64);
	B = (double*)mkl_calloc(n * 1 , sizeof(double), 64);
	C = (double*)mkl_calloc(1 * n , sizeof(double), 64);
	memcpy(A, _A, n * n * sizeof(double));
	memcpy(B, _B, n * 1 * sizeof(double));
	memcpy(C, _C, 1 * n * sizeof(double));

	Su = (double*)mkl_calloc(p * m , sizeof(double), 64);
	Sx = (double*)mkl_calloc(p * n , sizeof(double), 64);
	Wy = (double*)mkl_calloc(p * p , sizeof(double), 64);
	Wu = (double*)mkl_calloc(m * m , sizeof(double), 64);
	Kmpc = (double*)mkl_calloc(m * p , sizeof(double), 64);

	GenerateSu(Su, A, B, C, n, p, m);
	GenerateWeightMatrix(Wy, p, _wy);
	GenerateWeightMatrix(Wu, m, _wu);
	GenerateKmpc(Kmpc, Su, Wy, Wu, p, m);
	GenerateSx(Sx, A, C, n, p);
    
	x_store = (double*)mkl_calloc(n , sizeof(double), 64);
	U = (double*)mkl_calloc(m , sizeof(double), 64);
	ref_sub_mo = (double*)mkl_calloc(p , sizeof(double), 64);
	x = (double*)mkl_calloc(n , sizeof(double), 64);
	delta_x = (double*)mkl_calloc(n , sizeof(double), 64);
}


MPController::~MPController()
{
	mkl_free(A);
	mkl_free(B);
	mkl_free(C);
	mkl_free(Su);
	mkl_free(Sx);
	mkl_free(Wy);
	mkl_free(Wu);
	mkl_free(Kmpc);
	mkl_free(x_store);
	mkl_free(x);
	mkl_free(delta_x);
	mkl_free(ref_sub_mo);
	mkl_free(U);
    delete kalmanFilter;
}


void MPController::InitKalmanFilter(const double* _P, const double* _Q, double r)
{
	kalmanFilter = new KalmanFilter(A, B, C, n, _P, _Q, r);
}

double MPController::Update(double mo, const double* ref, int refSize)
{
	if (refSize <= 0)
		return 0;

	if (refSize >= p)
	{
		for (int i = 0; i < p; i++)
			ref_sub_mo[i] = ref[i] - mo;
	}
	else
	{
		for (int i = 0; i < refSize; i++)
			ref_sub_mo[i] = ref[i] - mo;
		for (int i = refSize; i < p; i++)
			ref_sub_mo[i] = ref_sub_mo[refSize - 1];
	}

	kalmanFilter->Update(x, mo, mv_store);
	//update state x (x is overwritten)

	vdSub(n, x, x_store, delta_x);
	//delta_x=x-x_store;

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		p, 1, n, -1.0, Sx, n, delta_x, 1, 1.0, ref_sub_mo, 1);
	//E=ref-I*mo-Sx*delta_x;
	//where E is stored in ref_sub_mo

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		m, 1, p, 1.0, Kmpc, p, ref_sub_mo, 1, 0.0, U, 1);
	//U=Kmpc*E;
	//PrintMatrix(U, m, 1);
	double mv = mv_store + U[0];
	//mv=mv_store+U(1);


	swap(&x_store, &x);
	mv_store = mv;
	return mv;
}