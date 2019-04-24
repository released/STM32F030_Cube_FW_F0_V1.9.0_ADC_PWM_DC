
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

struct t_pida
{
	float SetSpeed;
	float ActualSpeed;
	float err;
	float err_last;	
	float Kp,Ki,Kd;
	
	#if defined (PID_INCREMENTAL)
	float err_next;
	#endif

	#if defined (PID_ANTI_WINDUP)
	float voltage;
	float integral;
	float umax;
	float umin;
	#endif
}pida;

struct t_pidb
{
	float SetSpeed;
	float ActualSpeed;
	float err;
	float err_last;	
	float Kp,Ki,Kd;
	
	#if defined (PID_INCREMENTAL)
	float err_next;
	#endif

	#if defined (PID_ANTI_WINDUP)
	float voltage;
	float integral;
	float umax;
	float umin;
	#endif
}pidb;

/* Private functions ---------------------------------------------------------*/

void PIDb_init(float Kp , float Ki , float Kd)
{
    pidb.SetSpeed = 0.0;
    pidb.ActualSpeed = 0.0;
    pidb.err = 0.0;
    pidb.err_last = 0.0;
	pidb.Kp = Kp;
	pidb.Ki = Ki;
	pidb.Kd = Kd;	

	#if defined (PID_INCREMENTAL)	
	pidb.err_next = 0.0;
	#endif 

	#if defined (PID_ANTI_WINDUP)	
	pidb.voltage = 0.0;
	pidb.integral = 0.0;
	pidb.umax = 400;
	pidb.umin = -200; 
	#endif 
}

float PIDb_realize(float speed)
{

	#if defined (PID_INCREMENTAL)
	float incrementSpeed;

	pidb.SetSpeed=speed;
	pidb.err=pidb.SetSpeed-pidb.ActualSpeed;	
    incrementSpeed = pidb.Kp*(pidb.err-pidb.err_next) + pidb.Ki*pidb.err + pidb.Kd*(pidb.err - 2*pidb.err_next + pidb.err_last);
    pidb.ActualSpeed += incrementSpeed;
    pidb.err_last = pidb.err_next;
    pidb.err_next = pidb.err;
	#endif

	#if defined (PID_ANTI_WINDUP)
	int index;	

	pidb.SetSpeed=speed;
	pidb.err=pidb.SetSpeed-pidb.ActualSpeed;	
	if (pidb.ActualSpeed>pidb.umax)
	{
		if(abs(pidb.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pidb.err<0)
			{
				pidb.integral+=pidb.err;
			}
		}
	}
	else if (pidb.ActualSpeed<pidb.umin)
	{
		if(abs(pidb.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pidb.err>0)
			{
				pidb.integral+=pidb.err;
			}
		}
	}
	else
	{
		if(abs(pidb.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			pidb.integral+=pidb.err;
		}
	}
	
	pidb.voltage=pidb.Kp*pidb.err+index*pidb.Ki*pidb.integral+pidb.Kd*(pidb.err-pidb.err_last);
	pidb.err_last=pidb.err;
	pidb.ActualSpeed=pidb.voltage*1.0;
	#endif
	
	return pidb.ActualSpeed;
}

void PIDb_calculate(int timer , float target)
{
    int cnt = 0;
  	float speed = 0;

	while(cnt<timer)
    {
        speed = PIDb_realize(target);
        printf("%f\n",speed);
        cnt++;
    }	
}


void PIDa_init(float Kp , float Ki , float Kd)
{
    pida.SetSpeed = 0.0;
    pida.ActualSpeed = 0.0;
    pida.err = 0.0;
    pida.err_last = 0.0;
	pida.Kp = Kp;
	pida.Ki = Ki;
	pida.Kd = Kd;	

	#if defined (PID_INCREMENTAL)	
	pida.err_next = 0.0;
	#endif 

	#if defined (PID_ANTI_WINDUP)	
	pida.voltage = 0.0;
	pida.integral = 0.0;
	pida.umax = 400;
	pida.umin = -200; 
	#endif 
}

float PIDa_realize(float speed)
{

	#if defined (PID_INCREMENTAL)
	float incrementSpeed;

	pida.SetSpeed=speed;
	pida.err=pida.SetSpeed-pida.ActualSpeed;	
    incrementSpeed = pida.Kp*(pida.err-pida.err_next) + pida.Ki*pida.err + pida.Kd*(pida.err - 2*pida.err_next + pida.err_last);
    pida.ActualSpeed += incrementSpeed;
    pida.err_last = pida.err_next;
    pida.err_next = pida.err;
	#endif

	#if defined (PID_ANTI_WINDUP)
	int index;	

	pida.SetSpeed=speed;
	pida.err=pida.SetSpeed-pida.ActualSpeed;	
	if (pida.ActualSpeed>pida.umax)
	{
		if(abs(pida.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pida.err<0)
			{
				pida.integral+=pida.err;
			}
		}
	}
	else if (pida.ActualSpeed<pida.umin)
	{
		if(abs(pida.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pida.err>0)
			{
				pida.integral+=pida.err;
			}
		}
	}
	else
	{
		if(abs(pida.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			pida.integral+=pida.err;
		}
	}
	
	pida.voltage=pida.Kp*pida.err+index*pida.Ki*pida.integral+pida.Kd*(pida.err-pida.err_last);
	pida.err_last=pida.err;
	pida.ActualSpeed=pida.voltage*1.0;
	#endif
	
	return pida.ActualSpeed;
}

void PIDa_calculate(int timer , float target)
{
    int cnt = 0;
  	float speed = 0;

	while(cnt<timer)
    {
        speed = PIDa_realize(target);
        printf("%f\n",speed);
        cnt++;
    }	
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
