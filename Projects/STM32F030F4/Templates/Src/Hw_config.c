/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    	TIM14_Timer_Handle;

TIM_HandleTypeDef    	TIM3_PWM_Handle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef 		TIM3_PWM1_sConfig;
TIM_OC_InitTypeDef 		TIM3_PWM2_sConfig;

/* Counter Prescaler value */
uint32_t TIM3_PWM_PrescalerValue = 0;

/* UART handler declaration */
UART_HandleTypeDef 		UartHandle;

ADC_HandleTypeDef       	AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef  	ADC_sConfig;

/* Variable containing ADC conversions data */
uint16_t   				aADCxConvertedData[ADC_CH];

uint16_t   				adcRawData_Target = 0;
uint16_t   				adcRawData_Feedback = 0;

uint16_t 				movingAverage_Target = 0;
uint32_t 				movingAverageSum_Target = 0;

uint16_t 				movingAverage_Feedback = 0;
uint32_t 				movingAverageSum_Feedback = 0;

uint16_t 				adc2Duty_Target = 0;
uint16_t 				adc2Duty_Feedback = 0;

uint8_t 				ADC1_DataReady = 0;
uint8_t 				ADC2_DataReady = 0;

ADC_DataState_TypeDef 	ADC1_DataState = ADC_DataState_DEFAULT;
ADC_DataState_TypeDef 	ADC2_DataState = ADC_DataState_DEFAULT;
/* Private functions ---------------------------------------------------------*/

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

//uint32_t ABS(uint32_t x)
//{
//	uint32_t data = 0;
//
//	data = (x) > 0 ? (x) : -(x);
//	
//	return data;
//}

void ADC_Feedback_ModifiedMovingAverage(uint16_t data)
{
	static uint16_t cnt = 0;

	if (ADC2_DataReady)
	{
		ADC2_DataReady = 0;
		
//		printf("data : %d\r\n" , data);
		
		switch(ADC2_DataState)
		{
			case ADC_DataState_AVERAGE:
				movingAverageSum_Feedback += data;

				if (cnt++ >= (ADC_SAMPLE_COUNT-1))
				{
					cnt = 0;
					movingAverage_Feedback = movingAverageSum_Feedback >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;;
					ADC2_DataState = ADC_DataState_MMA;
				}			
				break;
				
			case ADC_DataState_MMA:
				movingAverageSum_Feedback -=  movingAverage_Feedback;
				movingAverageSum_Feedback +=  data;
				movingAverage_Feedback = movingAverageSum_Feedback >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;
	
//				printf("Average : %d\r\n" , movingAverage);
				break;				
		}

	}	
}

void ADC_Target_ModifiedMovingAverage(uint16_t data)
{
	static uint16_t cnt = 0;

	if (ADC1_DataReady)
	{
		ADC1_DataReady = 0;
		
//		printf("data : %d\r\n" , data);
		
		switch(ADC1_DataState)
		{
			case ADC_DataState_AVERAGE:
				movingAverageSum_Target += data;

				if (cnt++ >= (ADC_SAMPLE_COUNT-1))
				{
					cnt = 0;
					movingAverage_Target = movingAverageSum_Target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;;
					ADC1_DataState = ADC_DataState_MMA;
				}			
				break;
				
			case ADC_DataState_MMA:
				movingAverageSum_Target -=  movingAverage_Target;
				movingAverageSum_Target +=  data;
				movingAverage_Target = movingAverageSum_Target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;
	
//				printf("Average : %d\r\n" , movingAverage);
				break;				
		}

	}	
}

uint16_t ADC_ConvertChannel(ADC_SRC_TypeDef type)
{	
	__IO uint16_t adc_value = 0;
	__IO uint16_t duty_value = 0;
	uint16_t	currentADCValue = 0;

	switch(type)
	{
		case ADC_TARGET:
			adcRawData_Target = (aADCxConvertedData[ADC_TARGET]>>1)<<1;
			ADC1_DataReady = 1;	
		
			ADC_Target_ModifiedMovingAverage(adcRawData_Target);
			adc_value = movingAverage_Target;
			
			if (adc_value <= ADC_CONVERT_TARGET)
			{
				adc_value = ADC_CONVERT_TARGET;
			}

			if (adc_value >= ADC_RESOLUTION)
			{
				adc_value = ADC_RESOLUTION;
			}

			duty_value = (uint16_t)((DUTY_MIN) + ((adc_value - ADC_TARGET)/ADC_SUB_TARGET));

			if (duty_value <= DUTY_MIN)	//10
			{
				duty_value = DUTY_MIN;	//10
			}

			if (duty_value >= DUTY_MAX)
			{
				duty_value = DUTY_MAX;
			}	
			break;

		case ADC_FEEDBACK:
			adcRawData_Feedback = aADCxConvertedData[ADC_FEEDBACK];
			ADC2_DataReady = 1;	
		
			ADC_Feedback_ModifiedMovingAverage(adcRawData_Feedback);
			adc_value = movingAverage_Feedback;
			
			if (adc_value <= ADC_CONVERT_FEEDBACK)
			{
				adc_value = ADC_CONVERT_FEEDBACK;
			}

			if (adc_value >= ADC_RESOLUTION)
			{
				adc_value = ADC_RESOLUTION;
			}

			duty_value = (uint16_t)((DUTY_MIN) + ((adc_value - ADC_CONVERT_FEEDBACK)/ADC_SUB_FEEDBACK));
			
			if (duty_value <= DUTY_MIN)	//10
			{
				duty_value = DUTY_MIN;	//10
			}

			if (duty_value >= DUTY_MAX)
			{
				duty_value = DUTY_MAX;
			}	
			break;

			
	}

	return duty_value;
	
}

void PWM_Combo(void)
{
	__HAL_TIM_SET_COMPARE(&TIM3_PWM_Handle,TIM_CHANNEL_1,(DUTY_MAX*adc2Duty_Target)>>12);
//	__HAL_TIM_SET_COMPARE(&TIM3_PWM_Handle,TIM_CHANNEL_2,pwm2);	

//	printf("feedback : 0x%4X,0x%4X , target : %3d(0x%4X)\r\n",movingAverage_Feedback,adcRawData_Feedback,adc2Duty_Target,adcRawData_Target);
//	printf(">>4 : 0x%4X , >>5 : 0x%4X\r\n",movingAverageSum_Target>>4 , (movingAverageSum_Target>>5)<<1);

}

/*
	PA6 : TIM3_CH1
	PA7 : TIM3_CH2
	
	PB1 : TIM3_CH4

	TIMx Frequency =? TIMxCLK / ((TIM_Period + 1) * (TIM_Prescaler + 1))
	
	Target Freq : 4K
	ARR : 999
	PSC : 3	
*/

void PWM_Config(void)
{
	//Avery.20180418 , EM8801 only accept 200~400
	uint32_t TIMx_counter_clock = 200000;	//4000000;	//16000000;
	
	/* Compute the prescaler value to have TIM1 counter clock equal to 16000000 Hz */
	TIM3_PWM_PrescalerValue = (uint32_t)(SystemCoreClock / TIMx_counter_clock) - 1;	//target : 120

	/*##-1- Configure the TIM peripheral #######################################*/
	/* -----------------------------------------------------------------------
	TIM1 Configuration: generate 4 PWM signals with 4 different duty cycles.

	In this example TIM1 input clock (TIM1CLK) is set to APB1 clock (PCLK1),
	since APB1 prescaler is equal to 1.
	  TIM1CLK = PCLK1
	  PCLK1 = HCLK
	  => TIM1CLK = HCLK = SystemCoreClock

	To get TIM1 counter clock at 16 MHz, the prescaler is computed as follows:
	   Prescaler = (TIM1CLK / TIM1 counter clock) - 1
	   Prescaler = ((SystemCoreClock) /16 MHz) - 1

	To get TIM1 output clock at 24 KHz, the period (ARR)) is computed as follows:
	   ARR = (TIM1 counter clock / TIM1 output clock) - 1
	       = 665

	TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
	TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
	TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
	TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%

	Note:
	 SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
	 Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	 variable value. Otherwise, any configuration based on this variable will be incorrect.
	 This variable is updated in three ways:
	  1) by calling CMSIS function SystemCoreClockUpdate()
	  2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	  3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	----------------------------------------------------------------------- */

	/* Initialize TIMx peripheral as follows:
	   + Prescaler = (SystemCoreClock / 16000000) - 1
	   + Period = (666 - 1)
	   + ClockDivision = 0
	   + Counter direction = Up
	*/

	/*
		Freq : 	2KHz->160KHz
		16M / PERIOD_VALUE = 2K to 160K

		Duty : 
		PERIOD_VALUE * (percent/100)
	*/
	
	TIM3_PWM_Handle.Instance = TIM3;

	TIM3_PWM_Handle.Init.Prescaler         		= TIM3_PWM_PrescalerValue;
	TIM3_PWM_Handle.Init.Period            		= PERIOD_VALUE;
	TIM3_PWM_Handle.Init.ClockDivision     		= 0;
	TIM3_PWM_Handle.Init.CounterMode      		= TIM_COUNTERMODE_UP;
	TIM3_PWM_Handle.Init.RepetitionCounter 		= 0;
	TIM3_PWM_Handle.Init.AutoReloadPreload 		= TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&TIM3_PWM_Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	TIM3_PWM1_sConfig.OCMode       			= TIM_OCMODE_PWM1;
	TIM3_PWM1_sConfig.OCPolarity   				= TIM_OCPOLARITY_HIGH;
	TIM3_PWM1_sConfig.OCFastMode   			= TIM_OCFAST_DISABLE;
	TIM3_PWM1_sConfig.OCNPolarity  				= TIM_OCNPOLARITY_LOW;
	TIM3_PWM1_sConfig.OCNIdleState 				= TIM_OCNIDLESTATE_RESET;
	TIM3_PWM1_sConfig.OCIdleState  				= TIM_OCIDLESTATE_SET;

	TIM3_PWM2_sConfig.OCMode       			= TIM_OCMODE_PWM2;
	TIM3_PWM2_sConfig.OCPolarity   				= TIM_OCPOLARITY_LOW;
	TIM3_PWM2_sConfig.OCFastMode   			= TIM_OCFAST_DISABLE;
	TIM3_PWM2_sConfig.OCNPolarity  				= TIM_OCNPOLARITY_HIGH;
	TIM3_PWM2_sConfig.OCNIdleState 				= TIM_OCNIDLESTATE_RESET;
	TIM3_PWM2_sConfig.OCIdleState  				= TIM_OCIDLESTATE_SET;

	/* Set the pulse value for channel 3 */
	TIM3_PWM1_sConfig.Pulse 					= PULSE_VALUE;	//PULSE3_VALUE;
	if (HAL_TIM_PWM_ConfigChannel(&TIM3_PWM_Handle, &TIM3_PWM1_sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&TIM3_PWM_Handle, &TIM3_PWM1_sConfig, TIM_CHANNEL_4) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}
	

	TIM3_PWM2_sConfig.Pulse 					= PULSE_VALUE;	//PULSE3_VALUE;
	if (HAL_TIM_PWM_ConfigChannel(&TIM3_PWM_Handle, &TIM3_PWM2_sConfig, TIM_CHANNEL_2) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/*##-3- Start PWM signals generation #######################################*/
	/* Start channel 1 */
	if (HAL_TIM_PWM_Start(&TIM3_PWM_Handle, TIM_CHANNEL_1) != HAL_OK)
	{
		/* PWM Generation Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&TIM3_PWM_Handle, TIM_CHANNEL_2) != HAL_OK)
	{
		/* PWM Generation Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&TIM3_PWM_Handle, TIM_CHANNEL_4) != HAL_OK)
	{
		/* PWM Generation Error */
		Error_Handler();
	}
	
}

void ADC_DMA_Config(void)
{
	AdcHandle.Instance          			= ADC1;
	if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
	{
		/* ADC de-initialization Error */
		Error_Handler();
	}

	AdcHandle.Init.ClockPrescaler     	   	= ADC_CLOCK_SYNC_PCLK_DIV2;      /* Synchronous clock mode, input ADC clock with prscaler 2 */

	AdcHandle.Init.Resolution            	= ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
	AdcHandle.Init.DataAlign             	= ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
	AdcHandle.Init.ScanConvMode         	= ADC_SCAN_DIRECTION_FORWARD;    /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	AdcHandle.Init.EOCSelection          	= ADC_EOC_SEQ_CONV;           /* EOC flag picked-up to indicate conversion end */
	AdcHandle.Init.LowPowerAutoPowerOff 	= DISABLE;
	AdcHandle.Init.LowPowerAutoWait     	= DISABLE;                       /* Auto-delayed conversion feature disabled */
	AdcHandle.Init.ContinuousConvMode    	= ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
	AdcHandle.Init.DiscontinuousConvMode 	= DISABLE;                       /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.ExternalTrigConv      	= ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	AdcHandle.Init.ExternalTrigConvEdge  	= ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	AdcHandle.Init.DMAContinuousRequests 	= ENABLE;                        /* ADC DMA continuous request to match with DMA circular mode */
	AdcHandle.Init.Overrun               	= ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
	AdcHandle.Init.SamplingTimeCommon 	= ADC_SAMPLETIME_55CYCLES_5;

	/* Initialize ADC peripheral according to the passed parameters */
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		Error_Handler();
	}

	/* ### - 2 - Start calibration ############################################ */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
	{
		Error_Handler();
	}

	/* ### - 3 - Channel configuration ######################################## */
	ADC_sConfig.Channel      = ADC_CHANNEL_4;               /* Channel to be converted */
	ADC_sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&AdcHandle, &ADC_sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	ADC_sConfig.Channel      = ADC_CHANNEL_5;               /* Channel to be converted */
	if (HAL_ADC_ConfigChannel(&AdcHandle, &ADC_sConfig) != HAL_OK)
	{
		Error_Handler();
	}	

	/* ### - 4 - Start conversion in DMA mode ################################# */
	if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)aADCxConvertedData, ADC_CH) != HAL_OK)
	{
		Error_Handler();
	}

	ADC1_DataState = ADC_DataState_AVERAGE;
	ADC2_DataState = ADC_DataState_AVERAGE;	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{	

		#if defined (DEBUG)
//		static uint16_t cnt_timer = 0;			
//		static uint16_t tmp = 0;

		//insert application for TIMER (1ms)
//		if (cnt_timer++ >= 10)
//		{
//			cnt_timer = 0;			
//			printf("%s : %4d , 0x%4X , 0x%4X , 0x%4X , 0x%4X\r\n",__FUNCTION__,tmp++ ,adc2Duty_Target,currentADCValue, aADCxConvertedData[0],aADCxConvertedData[1]);
//			printf("%s : %4d , 0x%4X \r\n",__FUNCTION__,tmp++ ,PWM_Raise_GetFlag());
		
//		}
		#endif

	
		#if defined (ENABLE_PWM_NORMAL)
//		static uint16_t cnt_pwm = 0;	
		static uint16_t cnt_adc_temperature = 0;	
//		static uint16_t cnt_adc_brightness = 0;
	
		if (cnt_adc_temperature++ >= ADC_SAMPLETIME_MS)
		{
			cnt_adc_temperature = 0;
   			adc2Duty_Target = ADC_ConvertChannel(ADC_TARGET);
		}

//		if (cnt_adc_brightness++ >= ADC_SAMPLETIME_MS)
//		{
//			cnt_adc_brightness = 0;
//   			adc2Duty_Feedback = ADC_ConvertChannel(ADC_FEEDBACK);
//		}
		
//		if (cnt_pwm++ >= PWM_NORMAL_CHANGETIME_MS)	//	3
//		{
//			cnt_pwm = 0;
//			PWM_Raise_IRQHandler(target_pwm1_irq,target_pwm2_irq);
//		}
		#endif	
		
	}
}

void TIM14_Config(void)	//1ms
{
//	uint32_t	uTimPrescalerValue = 0;

	/* Compute the prescaler value to have TIMx counter clock equal to 1K Hz */
//	uTimPrescalerValue = (uint32_t)(SystemCoreClock / 1000) - 1;

	/*##-1- Configure the TIM peripheral #######################################*/ 
	/* Set TIMx instance */
	TIM14_Timer_Handle.Instance = TIM14;

	/* Initialize TIMx peripheral as follow:
	   + Period = 
	   + Prescaler = SystemCoreClock/10000 Note that APB clock = TIMx clock if
	                 APB prescaler = 1.
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	TIM14_Timer_Handle.Init.Period = (1000 -1);
	TIM14_Timer_Handle.Init.Prescaler = (48 -1);	//uTimPrescalerValue;
	TIM14_Timer_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM14_Timer_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM14_Timer_Handle.Init.RepetitionCounter = 0;
	TIM14_Timer_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	if(HAL_TIM_Base_Init(&TIM14_Timer_Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if(HAL_TIM_Base_Start_IT(&TIM14_Timer_Handle) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
}

void USART_Config(void)
{

#if defined (STM32F030x8)
	UartHandle.Instance          	= USART2;
#elif defined (STM32F030x6)
	UartHandle.Instance          	= USART1;
#endif

	UartHandle.Init.BaudRate     	= 115200;
	UartHandle.Init.WordLength   	= UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     	= UART_STOPBITS_1;
	UartHandle.Init.Parity       	= UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    	= UART_HWCONTROL_NONE;
	UartHandle.Init.Mode        	= UART_MODE_TX_RX;
	UartHandle.Init.OverSampling 	= UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler(); 
	}

	/* Output a message on Hyperterminal using printf function */
	printf("\r\nUART Printf Example: retarget the C library printf function to the UART\r\n");

	printf("HAL_RCC_GetSysClockFreq = %d\r\n",HAL_RCC_GetSysClockFreq());
 	printf("HAL_RCC_GetHCLKFreq = %d\r\n",HAL_RCC_GetHCLKFreq());
	printf("HAL_RCC_GetPCLK1Freq = %d\r\n",HAL_RCC_GetPCLK1Freq());
	printf("SystemCoreClock = %d\r\n",SystemCoreClock);
	
}


