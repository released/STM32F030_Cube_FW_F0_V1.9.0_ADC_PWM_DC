/**
  ******************************************************************************
  * @file    Templates/Src/stm32f0xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
 
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

	GPIO_InitTypeDef GPIO_InitStruct;
  	static DMA_HandleTypeDef         DmaHandle;

	/* Peripheral clock enable */
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/**ADC GPIO Configuration    
	PA4     ------> ADC_IN4
	PA5     ------> ADC_IN5 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* ADC1 DMA Init */
	/* ADC Init */
	DmaHandle.Instance = DMA1_Channel1;
	DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
	DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
	DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	DmaHandle.Init.Mode = DMA_CIRCULAR;
	DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;

	HAL_DMA_DeInit(&DmaHandle);
	HAL_DMA_Init(&DmaHandle);

	/* Associate the DMA handle */
	__HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

	/* NVIC configuration for DMA Input data interrupt */
//	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);  

//	HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(ADC1_IRQn);

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

	__HAL_RCC_ADC1_FORCE_RESET();
	__HAL_RCC_ADC1_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA4     ------> ADC_IN4
    PA5     ------> ADC_IN5 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);//GPIO_PIN_5

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

//	/* Disable the NVIC configuration for DMA interrupt */
//	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

//	/* Disable the NVIC configuration for ADC interrupt */
//	HAL_NVIC_DisableIRQ(ADC1_IRQn);

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
#if defined (STM32F030x8)
	if(huart->Instance == USART2)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		/**USART1 GPIO Configuration    
		PA2     ------> USART2_TX
		PA3     ------> USART2_RX 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}

	
#elif defined (STM32F030x6)
	if(huart->Instance == USART1)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		/**USART1 GPIO Configuration    
		PA2     ------> USART1_TX
		PA3     ------> USART1_RX 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}	
#endif
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

#if defined (STM32F030x8)
	if(huart->Instance == USART2)
	{
		__HAL_RCC_USART2_FORCE_RESET();
		__HAL_RCC_USART2_RELEASE_RESET();

		/* Peripheral clock disable */
		__HAL_RCC_USART2_CLK_DISABLE();

		/**USART1 GPIO Configuration    
		PA2     ------> USART2_TX
		PA3     ------> USART2_RX 
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
	}
	
#elif defined (STM32F030x6)
	if(huart->Instance == USART1)
	{
		__HAL_RCC_USART1_FORCE_RESET();
		__HAL_RCC_USART1_RELEASE_RESET();

		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration    
		PA2     ------> USART1_TX
		PA3     ------> USART1_RX 
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
	}	
#endif


}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
 	 GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIMx Peripheral clock enable */
		__HAL_RCC_TIM14_CLK_ENABLE();

		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);

		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM14_IRQn);
	}
}


void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspDeInit 0 */

  /* USER CODE END TIM14_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM14_CLK_DISABLE();

    /* TIM14 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM14_IRQn);
  /* USER CODE BEGIN TIM14_MspDeInit 1 */

  /* USER CODE END TIM14_MspDeInit 1 */
  }

}

/**
  * @brief  Initializes the PPP MSP.
  * @param  None
  * @retval None
  */
//void HAL_PPP_MspInit(void)
//{
 
//}

/**
  * @brief  DeInitializes the PPP MSP.
  * @param  None  
  * @retval None
  */
//void HAL_PPP_MspDeInit(void)
//{

//}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
