#include "pid.h"

/* ΪPID���������ڴ棬��Χָ��pp��ָ�� */
void PIDInit (PID *pp)   
{
	memset(pp, 0, sizeof(PID)); /* �ֽڵ�����ȫ������Ϊchָ����ASCIIֵ����Ĵ�С�ɵ���������ָ�� */
}

/* ��׼��λ��ʽPID */
double PIDCalc(PID *pp, double NextPoint)   
{
	double dError,                              /* ��ǰ΢�� */
				 Error;                               /* ƫ�� */
	Error = pp->SetPoint - NextPoint;           /* ƫ��ֵ=�趨ֵ-����ֵ����ǰֵ��*/
	pp->SumError += Error;                      /* ����=����+ƫ��  --ƫ����ۼ� */
	dError = pp->LastError - pp->PrevError;     /* ��ǰ΢�� = ������ - ֮ǰ��� */
	pp->PrevError = pp->LastError;              /* ���¡�֮ǰ�� */
	pp->LastError = Error;                      /* ���¡������ */
	return (pp->Kp * Error                      /* ������ = �������� * ƫ�� */
			+   pp->Ki *  pp->SumError              /* ������ = ���ֳ��� * ������ */
			+   pp->Kd * dError                     /* ΢���� = ΢�ֳ��� * ��ǰ΢�� */
				 );
}

/* �ٶȻ� */
double PID_V(PID *pp, double NextPoint)   
{
	double dError,                              /* ��ǰ΢�� */
				 Error;                               /* ƫ�� */
	Error = pp->SetPoint - NextPoint;           /* ƫ��ֵ=�趨ֵ-����ֵ����ǰֵ��*/
	pp->SumError += Error;                      /* ����=����+ƫ��  --ƫ����ۼ� */
	dError = pp->LastError - pp->PrevError;     /* ��ǰ΢�� = ������ - ֮ǰ��� */
	pp->PrevError = pp->LastError;              /* ���¡�֮ǰ�� */
	pp->LastError = Error;                      /* ���¡������ */
	return (pp->Kp * Error                      /* ������ = �������� * ƫ�� */
			+   pp->Ki *  pp->SumError              /* ������ = ���ֳ��� * ������ */
				 );
}

/* �ǶȻ� */
double PID_A(PID *pp, double NextPoint)   
{
	double dError,                              /* ��ǰ΢�� */
				 Error;                               /* ƫ�� */
	Error = pp->SetPoint - NextPoint;           /* ƫ��ֵ=�趨ֵ-����ֵ����ǰֵ��*/
	pp->SumError += Error;                      /* ����=����+ƫ��  --ƫ����ۼ� */
	dError = pp->LastError - pp->PrevError;     /* ��ǰ΢�� = ������ - ֮ǰ��� */
	pp->PrevError = pp->LastError;              /* ���¡�֮ǰ�� */
	pp->LastError = Error;                      /* ���¡������ */
	return (pp->Kp * (vPIDOutput + mechanicalZero - NextPoint)  /* ������ = �������� * ƫ�� */
			+   pp->Kd * dError                     /* ΢���� = ΢�ֳ��� * ��ǰ΢�� */
				 );
}

/* ���ٶȻ� */
double PID_W(PID *pp, double NextPoint)   
{
	double dError,                              /* ��ǰ΢�� */
				 Error;                               /* ƫ�� */
	Error = pp->SetPoint - NextPoint;           /* ƫ��ֵ=�趨ֵ-����ֵ����ǰֵ��*/
	pp->SumError += Error;                      /* ����=����+ƫ��  --ƫ����ۼ� */
	dError = pp->LastError - pp->PrevError;     /* ��ǰ΢�� = ������ - ֮ǰ��� */
	pp->PrevError = pp->LastError;              /* ���¡�֮ǰ�� */
	pp->LastError = Error;                      /* ���¡������ */
	return (pp->Kp * (aPIDOutput - NextPoint)                      /* ������ = �������� * ƫ�� */
			+   pp->Ki *  pp->SumError              /* ������ = ���ֳ��� * ������ */
				 );
}
