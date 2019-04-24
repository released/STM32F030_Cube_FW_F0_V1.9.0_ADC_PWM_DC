
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define abs(X)  			((X) > 0 ? (X) : -(X))

//#define PID_INCREMENTAL 
#define PID_ANTI_WINDUP

/* Exported functions ------------------------------------------------------- */

void PIDa_init(float Kp , float Ki , float Kd);
float PIDa_realize(float speed);
void PIDa_calculate(int timer , float target);

void PIDb_init(float Kp , float Ki , float Kd);
float PIDb_realize(float speed);
void PIDb_calculate(int timer , float target);

#endif /* __PID_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
