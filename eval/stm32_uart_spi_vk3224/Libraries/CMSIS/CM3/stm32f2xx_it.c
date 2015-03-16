/**
  ******************************************************************************
  * @file     DAC/DAC_SignalsGeneration/stm32f2xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "VK3224.h"
//#include "stm32_eval.h"

/** @addtogroup STM32F2xx_StdPeriph_Examples
  * @{
  */
/** @addtogroup DAC_SignalsGeneration
  * @{
  */ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint8_t SelectedWavesForm, KeyPressed;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
/**
  * @brief  This function handles External line 15 interrupt request.
  * @param  None
  * @retval None
  */
#if 0
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(STM322xG_EVAL) != RESET)
  { 
    /* Change the wave */
    KeyPressed = 1;

    /* Change the selected waves forms */
    SelectedWavesForm = !SelectedWavesForm;

    /* Clear the Right Button EXTI line pending bit */
    EXTI_ClearITPendingBit(KEY_BUTTON_EXTI_LINE);
  }
}
#endif

/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	#if 0
	uint32_t i=0;
	uint16_t readVal=0; 
	/* Disable general interrupts */
	__set_PRIMASK(1);
	
	/* UserButton usage activated*/ 
	//printf("exti0...\n");
	while ((VK32XX_IRQ_GPIO_PORT->IDR & VK32XX_IRQ_PIN) == 0 )
    {
		i++;
		if (i == 0x80000)
		{
			break;
		}
    }
	EXTI_ClearITPendingBit(EXTI_Line0);
	readVal=VK32XX_UART_ReceiveData( VKCOM4 );
	printf("r=%x\r\n",readVal);
	__set_PRIMASK(0);
	#else
	uint16_t dat, com;
	uint8_t num, i;
	if( EXTI_GetITStatus( VK32XX_IRQ_EXTI_LINE ) == SET )
	{
		printf("ex irq\r\n",dat);
		do
		{
			com = VK32XX_Get_IT_UARTx( );
			if( VK32XX_UART_GetITStatus( com, VK32XX_UART_IT_RFIEN ) == SET )
			{
		 		num = VK32XX_UART_Get_Num_RXFIFO( com );
				for( i=0; i< num; i++ )
				{
					dat = VK32XX_UART_ReceiveData( com );

					//while( USART_GetFlagStatus( USART1, USART_FLAG_TXE )  ==  RESET );
					//USART_SendData( USART1, dat );
					printf("c%xr-%x\r\n",com,dat);
				}
				VK32XX_UART_ClearITPendingBit( com, VK32XX_UART_IT_RFIEN );
			}
		}
		while( GPIO_ReadInputDataBit( VK32XX_IRQ_GPIO_PORT,  VK32XX_IRQ_PIN ) == 0 );
		EXTI_ClearITPendingBit( VK32XX_IRQ_EXTI_LINE );
	}
	#endif
}

/**
  * @}
  */ 
/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
