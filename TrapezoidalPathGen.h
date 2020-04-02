#pragma once
#include <math.h>

void TrapezoidalPathGen(double offset, double s, double v, double ts, double vmax, double amax, int bufferSize, double* path, double* velocity)
{
	if (v < 0)
	{
		TrapezoidalPathGen(-offset, -s, -v, ts, vmax, amax, bufferSize, path, velocity);
		for (int i = 0; i < bufferSize; i++)
		{
			path[i] = -path[i];
			velocity[i] = -velocity[i];
		}
	}
	else
	{
		double t_0_2_vm = vmax / amax;//从0加速至vmax的时间
		double x_0_2_vm = 1 / 2.0 * vmax * t_0_2_vm;//从0加速至vmax时走过的路程
		double t_v_2_vm = (vmax - v) / amax;//从v加速至vmax的时间
		double x_v_2_vm = 1 / 2.0 * (vmax + v) * t_v_2_vm;//从v加速至vmax时走过的路程
		double t_0_2_v = v / amax;//从0加速至v的时间
		double x_0_2_v = 1 / 2.0 * v * t_0_2_v;

		int pathSize = 0;

		if (s > x_v_2_vm + x_0_2_vm)
		{
			double t_pacc = t_v_2_vm;//正向加速
			double t_pconst = (s - x_v_2_vm - x_0_2_vm) / vmax;//正向匀速
			double t_pdeacc = t_0_2_vm;//正向减速
			double t_nacc = 0;//反向加速
			double t_nconst = 0;//反向匀速
			double t_ndeacc = 0;//反向减速

			if (round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts) <= bufferSize)
				pathSize = round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts);
			else
				pathSize = bufferSize;

			for (int i = 0; i < pathSize; i++)
			{
				auto t = (i + 1) * ts;
				if (t <= t_pacc)
				{
					path[i] = v * t + 1 / 2.0 * amax * t * t + offset;
					velocity[i] = v + amax * t;
				}
				else
					if (t < t_pacc + t_pconst)
					{
						t = t - t_pacc;
						path[i] = x_v_2_vm + vmax * t  + offset;
						velocity[i] = vmax;
					}
					else
					{
						t = t - t_pacc - t_pconst;
						path[i] = x_v_2_vm + vmax * t_pconst + vmax * t - 1 / 2.0 * amax * t * t  + offset;
						velocity[i] = vmax - amax * t;
					}
			}
		}
		//如果s稍短，则先加速然后减速到0
		else if (s <= x_v_2_vm + x_0_2_vm && s > x_0_2_v)
		{
			double t_pacc = sqrt((s + x_0_2_v) / amax) - t_0_2_v;//正向加速
			double t_pconst = 0;//正向匀速
			double t_pdeacc = sqrt((s + x_0_2_v) / amax);//正向减速
			double  t_nacc = 0;//反向加速
			double t_nconst = 0;//反向匀速
			double t_ndeacc = 0;//反向减速

			if (round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts) <= bufferSize)
				pathSize = round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts);
			else
				pathSize = bufferSize;

			for (int i = 0; i < pathSize; i++)
			{
				auto t = (i + 1) * ts;
				if (t <= t_pacc)
				{
					path[i] = v * t + 1 / 2.0 * amax * t * t + offset;
					velocity[i] = v + amax * t;
				}
				else
				{

					t = t - t_pacc;
					path[i] = v * t_pacc + 1 / 2.0 * amax * t_pacc * t_pacc + (v + t_pacc * amax) * t - 1 / 2.0 * amax * t * t + offset;
					velocity[i] = v + amax * t_pacc - amax * t;
				}
			}
		}
		//如果s很短甚至为负，则需要立即减速，减速到零后反向加速，然后减速到0
		else if (s > x_0_2_v - 2 * x_0_2_vm && s <= x_0_2_v)
		{
			double t_pacc = 0;//正向加速
			double t_pconst = 0;//正向匀速
			double t_pdeacc = t_0_2_v;//正向减速
			double t_nacc = sqrt((x_0_2_v - s) / amax);//反向加速
			double t_nconst = 0;//反向匀速
			double t_ndeacc = sqrt((x_0_2_v - s) / amax);//反向减速

			if (round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts) <= bufferSize)
				pathSize = round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts);
			else
				pathSize = bufferSize;
			for (int i = 0; i < pathSize; i++)
			{
				auto t = (i + 1) * ts;
				if (t <= t_pdeacc)
				{
					path[i] = v * t - 1 / 2.0 * amax * t * t + offset;
					velocity[i] = v - amax * t;
				}
				else if (t < t_pdeacc + t_nacc)
				{
					t = t - t_pdeacc;
					path[i] = x_0_2_v - 1 / 2.0 * amax * t * t + offset;
					velocity[i] = -amax * t;
				}
				else
				{
					t = t - t_pdeacc - t_nacc;
					path[i] = x_0_2_v - 1 / 2.0 * amax * t_nacc * t_nacc - (amax * t_nacc * t - 1 / 2.0 * amax * t * t) + offset;
					velocity[i] = -amax * t_nacc + amax * t;
				}
			}
		}
		//如果s负的很多，则需要立即减速，减速到零后反向加速到最大速度，然后匀速，然后减速到0
		else if (s <= x_0_2_v - 2 * x_0_2_vm)
		{
			double t_pacc = 0;//正向加速
			double t_pconst = 0;//正向匀速
			double t_pdeacc = t_0_2_v;//正向减速
			double t_nacc = t_0_2_vm;//反向加速
			double t_nconst = (x_0_2_v - s - 2 * x_0_2_vm) / vmax;//反向匀速
			double t_ndeacc = t_0_2_vm;//反向减速

			if (round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts) <= bufferSize)
				pathSize = round((t_pacc + t_pconst + t_pdeacc + t_nacc + t_nconst + t_ndeacc) / ts);
			else
				pathSize = bufferSize;
			for (int i = 0; i < pathSize; i++)
			{
				auto t = (i + 1) * ts;
				if (t <= t_pdeacc)
				{
					path[i] = v * t - 1 / 2.0 * amax * t * t + offset;
					velocity[i] = v - amax * t;
				}
				else if (t < t_pdeacc + t_nacc)
				{
					t = t - t_pdeacc;
					path[i] = x_0_2_v - 1 / 2.0 * amax * t * t + offset;
					velocity[i] = -amax * t;
				}
				else if (t < t_pdeacc + t_nacc + t_nconst)
				{
					t = t - t_pdeacc - t_nacc;
					path[i] = x_0_2_v - x_0_2_vm - vmax * t + offset;
					velocity[i] = -vmax;
				}
				else
				{
					t = t - t_pdeacc - t_nacc - t_nconst;
					path[i] = x_0_2_v - x_0_2_vm - vmax * t_nconst - (vmax * t - 1 / 2.0 * amax * t * t) + offset;
					velocity[i] = -vmax + amax * t;
				}
			}
		}

		if (pathSize > 0)
			for (int i = pathSize; i < bufferSize; i++)
			{
				path[i] = path[pathSize - 1];
				velocity[i] = 0;
			}
		else			
			for (int i = 0; i < bufferSize; i++)
		{
			path[i] = offset;
			velocity[i] = 0;
		}

	}
}