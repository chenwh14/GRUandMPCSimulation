#pragma once

class KalmanFilter
{
private:
	double* A;
	double* B;
	double* C;
	int n;
	double* P;
	double* Q;
	double r;
	double* x_store;
	double* x_based_on_last;
	double* P_;
	double* P_AT;
	double* P_CT;
	double* C_P_CT_R;
	double* K;
	double* I_KC;
public:
	KalmanFilter(const double* _A, const double* _B, const double* _C, int _n, const double* _P, const double* _Q, double _r);
	~KalmanFilter();

	void Update(double* x, double measured, double u_last);

};
