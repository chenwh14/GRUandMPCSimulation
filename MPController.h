#pragma once
#include "KalmanFilter.h"

class MPController
{
private:
	double* A;//system matrix
	double* B;//control matrix
	double* C;//output matrix
	int n;//order of the system
	int p;//prediction horizon
	int m;//control horizon
	double* Su;
	double* Sx;
	double* Kmpc;
	double* Wy;//weight of controller output
	double* Wu;//weight of control value chage

	double* x_store;
	double mv_store;

	KalmanFilter* kalmanFilter;

	double* ref_sub_mo;//avoid frequent memory re-allocate
	double* x;
	double* delta_x;
	double* U;
public:
	MPController(const double* _A, const double* _B, const double* _C, int _n, int _p, int _m, double _wy, double _wu);
	~MPController();
	void InitKalmanFilter(const double* _P, const double* _Q, double r);
	double Update(double mo,const double* ref, int refSize);
};