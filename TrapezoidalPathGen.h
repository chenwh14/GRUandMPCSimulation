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
		double t_0_2_vm = vmax / amax;//��0������vmax��ʱ��
		double x_0_2_vm = 1 / 2.0 * vmax * t_0_2_vm;//��0������vmaxʱ�߹���·��
		double t_v_2_vm = (vmax - v) / amax;//��v������vmax��ʱ��
		double x_v_2_vm = 1 / 2.0 * (vmax + v) * t_v_2_vm;//��v������vmaxʱ�߹���·��
		double t_0_2_v = v / amax;//��0������v��ʱ��
		double x_0_2_v = 1 / 2.0 * v * t_0_2_v;

		int pathSize = 0;

		if (s > x_v_2_vm + x_0_2_vm)
		{
			double t_pacc = t_v_2_vm;//�������
			double t_pconst = (s - x_v_2_vm - x_0_2_vm) / vmax;//��������
			double t_pdeacc = t_0_2_vm;//�������
			double t_nacc = 0;//�������
			double t_nconst = 0;//��������
			double t_ndeacc = 0;//�������

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
		//���s�Զ̣����ȼ���Ȼ����ٵ�0
		else if (s <= x_v_2_vm + x_0_2_vm && s > x_0_2_v)
		{
			double t_pacc = sqrt((s + x_0_2_v) / amax) - t_0_2_v;//�������
			double t_pconst = 0;//��������
			double t_pdeacc = sqrt((s + x_0_2_v) / amax);//�������
			double  t_nacc = 0;//�������
			double t_nconst = 0;//��������
			double t_ndeacc = 0;//�������

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
		//���s�ܶ�����Ϊ��������Ҫ�������٣����ٵ��������٣�Ȼ����ٵ�0
		else if (s > x_0_2_v - 2 * x_0_2_vm && s <= x_0_2_v)
		{
			double t_pacc = 0;//�������
			double t_pconst = 0;//��������
			double t_pdeacc = t_0_2_v;//�������
			double t_nacc = sqrt((x_0_2_v - s) / amax);//�������
			double t_nconst = 0;//��������
			double t_ndeacc = sqrt((x_0_2_v - s) / amax);//�������

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
		//���s���ĺܶ࣬����Ҫ�������٣����ٵ��������ٵ�����ٶȣ�Ȼ�����٣�Ȼ����ٵ�0
		else if (s <= x_0_2_v - 2 * x_0_2_vm)
		{
			double t_pacc = 0;//�������
			double t_pconst = 0;//��������
			double t_pdeacc = t_0_2_v;//�������
			double t_nacc = t_0_2_vm;//�������
			double t_nconst = (x_0_2_v - s - 2 * x_0_2_vm) / vmax;//��������
			double t_ndeacc = t_0_2_vm;//�������

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