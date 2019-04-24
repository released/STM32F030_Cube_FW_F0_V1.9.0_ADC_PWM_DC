/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>

//#include "pid.h"
/* Define config -------------------------------------------------------------*/

#define ENABLE_PWM_NORMAL
#define DEBUG

#if defined (DEBUG)
#define printf		printf
#else
#define printf		(void)
#endif

#define abs(x)  								((x) > 0 ? (x) : -(x))

#define PERIOD_VALUE       					(uint16_t)(1000 - 1)	//ARR	(1000 - 1)
#define PULSE_VALUE							(uint8_t)(0)			//DUTY = (TIMx_CCRy/ TIMx_ARR + 1)* 100 = TIMx_CCRy * 10
//#define DUTY_SET_TEMPERATURE(x)			(uint16_t)(x*10)	//(uint16_t)((x*(ARR_TEMPERATURE+1))/100)

#define DUTY_MAX							(uint16_t)(100)
#define DUTY_MIN							(uint16_t)(0)

#define ADC_CH   							(uint8_t) (2)

#define ADC_SAMPLETIME_MS					(uint16_t) (20)

#define ADC_RESOLUTION						((uint16_t)(4096u))
#define ADC_REF_VOLTAGE						((uint16_t)(3300u))	//(float)(3.3f)

#define ADC_MAX_TARGET						((uint16_t)(4095u))	//(float)(2.612f)
#define ADC_MIN_TARGET						((uint16_t)(0u))	//(float)(0.423f)

#define ADC_CONVERT_TARGET					(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 
#define ADC_SUB_TARGET						(float)((ADC_MAX_TARGET-ADC_MIN_TARGET)/(DUTY_MAX-DUTY_MIN)*(ADC_RESOLUTION/ADC_REF_VOLTAGE))//5.60505 
//#define ADCInputV_Sub						(float)	((ADC_MAX_BRIGHT-ADC_MIN_BRIGHT)/(DUTY_MAX-DUTY_MIN)) //0.02737 

#define ADC_MAX_FEEDBACK					(ADC_MAX_TARGET)
#define ADC_MIN_FEEDBACK					(ADC_MIN_TARGET)

#define ADC_CONVERT_FEEDBACK				(ADC_CONVERT_TARGET) 
#define ADC_SUB_FEEDBACK					(ADC_SUB_TARGET)

#define ADC_SAMPLE_COUNT 					(uint16_t) 	(16)		// 8
#define ADC_SAMPLE_POWER 					(uint8_t) 	(4)			//(5)	 	// 3	,// 2 ^ ?

#define PWM_NORMAL_CHANGETIME_MS		(uint16_t) (1)

#define PWM_INCREASE						(0)
#define PWM_DECREASE						(1)

#define ABS(X)  ((X) > 0 ? (X) : -(X)) 

typedef enum
{
	ADC_TARGET = 0 ,	

	ADC_FEEDBACK ,	
	
	ADC_DEFAULT

}ADC_SRC_TypeDef;

typedef enum
{
	ADC_DataState_AVERAGE = 0 ,
	ADC_DataState_MMA , 
	
	ADC_DataState_DEFAULT 	
}ADC_DataState_TypeDef;

/* Macro ---------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

void PWM_Combo(void);
void PWM_Config(void);

void ADC_DMA_Config(void);

void TIM14_Config(void);
void USART_Config(void);

#endif  /* __HW_CONFIG_H */

