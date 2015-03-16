#include "stm32f2xx.h"
#include "system_stm32f2xx.h"
#include "core_cm3.h"
#include "stm32f2xx_conf.h"
#include "stm322xg_eval.h"

#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_spi.h"
#include "misc.h"
#include "VK3224.h"
#include "stm32f2xx_exti.h"
#include "stm32f2xx_it.h"

#include <stdio.h>

extern uint16_t uartBaudRatePrescaler[16];

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	if (ch == '\n')
	{
		uint8_t rch = '\r';
		
		USART_SendData(EVAL_COM1, (uint8_t) rch);
	
		while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
		{}
	}
	
	USART_SendData(EVAL_COM1, (uint8_t) ch);
	
	while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
	{}
	
	return ch;
}

static void uart_init()
{
	/* USARTx configuration ------------------------------------------------------*/
	/* USARTx configured as follow:
	    - BaudRate = 9600 baud  
	    - Word Length = 8 Bits
	    - Two Stop Bit
	    - Odd parity
	    - Hardware flow control disabled (RTS and CTS signals)
	    - Receive and transmit enabled
	*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//USART_StopBits_2;	//;
	USART_InitStructure.USART_Parity = USART_Parity_No;//USART_Parity_Odd;// 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	STM_EVAL_COMInit(COM1, &USART_InitStructure);
	//USART_ITConfig(EVAL_COM1, USART_IT_TXE, ENABLE);
}

/* GPIO配置函数 ------------------------------------------------------------------*/
/*
**配置GPIOC_3,为推挽输出，速度为10Mhz，作为运行指示灯。
**配置GPIOA4、GPIOA5、GPIOA7为开漏复用输出，速度10Mhz；GPIOA6为下拉输入。
**配置GPIOA_9为推挽复用输出; 速度10Mhz;	GPIOA_10为下拉输入。
**配置GPIOA_2,为上拉输入，用于检测VK3224的中断。
*/
void GPIO_Configuration ( void )
{
	GPIO_InitTypeDef  GPIOC_InitStruct;
	GPIO_InitTypeDef  GPIOA_InitStruct;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);	//使能GPIOC时钟
	 
	GPIOC_InitStruct.GPIO_Pin  = GPIO_Pin_3 ;
    GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOC, &GPIOC_InitStruct ); 
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1   ,ENABLE);	//使能GAPIOA和USART1的时钟

	GPIOA_InitStruct.GPIO_Pin  =  GPIO_Pin_9 ;
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIOA_InitStruct );
	
	GPIOA_InitStruct.GPIO_Pin  =  GPIO_Pin_10 ;
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIOA_InitStruct );

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );	//使能SPI1的时钟；

	GPIOA_InitStruct.GPIO_Pin  =  GPIO_Pin_5 | GPIO_Pin_7;  //SPI_CLK和SPI_MOSI必须推挽输出
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIOA_InitStruct );

 	GPIOA_InitStruct.GPIO_Pin  =  GPIO_Pin_6;
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIOA_InitStruct );

	GPIOA_InitStruct.GPIO_Pin  =  GPIO_Pin_1;  
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIOA_InitStruct );

	//RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );

	GPIOA_InitStruct.GPIO_Pin  =  GPIO_Pin_2;  
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA_InitStruct.GPIO_Mode = GPIO_PuPd_NOPULL;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIOA_InitStruct );
		 
}
/********************************************************************************/





/* NVIC配置函数 ------------------------------------------------------------------*/
/*
** 配置1位抢占式优先级，3位副优先级
*/
void NVIC_Configuration( void )
{
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_1 );

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStruct );

	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStruct );
}

 void VK32XX_LowLevel_Init(void)
 {
   GPIO_InitTypeDef GPIO_InitStructure;
 
   /*!< Enable the SPI clock */
   VK32XX_SPI_CLK_INIT(VK32XX_SPI_CLK, ENABLE);
 
   /*!< Enable GPIO clocks */
   RCC_AHB1PeriphClockCmd(VK32XX_SPI_SCK_GPIO_CLK | VK32XX_SPI_MISO_GPIO_CLK | 
						  VK32XX_SPI_MOSI_GPIO_CLK | VK32XX_CS_GPIO_CLK, ENABLE);
   
   /*!< SPI pins configuration *************************************************/
 
   /*!< Connect SPI pins to AF5 */	
   GPIO_PinAFConfig(VK32XX_SPI_SCK_GPIO_PORT, VK32XX_SPI_SCK_SOURCE, VK32XX_SPI_SCK_AF);
   GPIO_PinAFConfig(VK32XX_SPI_MISO_GPIO_PORT, VK32XX_SPI_MISO_SOURCE, VK32XX_SPI_MISO_AF);
   GPIO_PinAFConfig(VK32XX_SPI_MOSI_GPIO_PORT, VK32XX_SPI_MOSI_SOURCE, VK32XX_SPI_MOSI_AF);
 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		 
   /*!< SPI SCK pin configuration */
   GPIO_InitStructure.GPIO_Pin = VK32XX_SPI_SCK_PIN;
   GPIO_Init(VK32XX_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
 
   /*!< SPI MOSI pin configuration */
   GPIO_InitStructure.GPIO_Pin =  VK32XX_SPI_MOSI_PIN;
   GPIO_Init(VK32XX_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
 
   /*!< SPI MISO pin configuration */
   GPIO_InitStructure.GPIO_Pin =  VK32XX_SPI_MISO_PIN;
   GPIO_Init(VK32XX_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
 
   /*!< Configure VK32XX Card CS pin in output pushpull mode ********************/
   GPIO_InitStructure.GPIO_Pin = VK32XX_CS_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(VK32XX_CS_GPIO_PORT, &GPIO_InitStructure);
 }



 /* SPI1配置函数 ------------------------------------------------------------------*/
/*
**全双工通信；工作在主机模式；数据位16位；时钟极性：低电平；时钟相位：第一个边缘。
*/
void SPI_Configuration ( void )
{
  SPI_InitTypeDef  SPI_InitStructure;

  VK32XX_LowLevel_Init();
    
  /*!< Deselect the FLASH: Chip Select high */
  VK32XX_CS_HIGH();

  /*!< SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(VK32XX_SPI, &SPI_InitStructure);

  /*!< Enable the VK32XX_SPI  */
  SPI_Cmd(VK32XX_SPI, ENABLE);  
}
/********************************************************************************/





/* EXTI2配置函数 ------------------------------------------------------------------*/
/*
**配置外部中断线2为下降沿触发中断。
*/
/*
void EXTI2_Configuration( void )
{
	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_StructInit( &EXTI_InitStruct );

	//GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource2 );
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);

	EXTI_Init( &EXTI_InitStruct );
}*/
/********************************************************************************/

/**
  * @brief  Configures EXTI Line8 (connected to PB8 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void VK32XX_EXTI_Configuration(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(VK32XX_IRQ_GPIO_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = VK32XX_IRQ_PIN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(VK32XX_IRQ_GPIO_PORT, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PB8 pin */
  SYSCFG_EXTILineConfig(VK32XX_IRQ_PORT_SOURCE, VK32XX_IRQ_PIN_SOURCE);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = VK32XX_IRQ_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}





/**********************************
 * 函数名：Init_vk3224();
 * 描述  ：初始化VK3224四个串口的工作状态
 * 输入  : 无
 * 输出  ：无
 * 举例  ：无
 * 注意  ：无
*************************************/
uint16_t Init_vk3224_getBaudRateSet(uint32_t baudRate)
{
	uint16_t prescalerIdx,prescaler=vk32xx_fr/baudRate;
	
	for(prescalerIdx=0;prescalerIdx<16;prescalerIdx++)
	{
		if(uartBaudRatePrescaler[prescalerIdx]==prescaler)
		{
			printf("prescalerIdx ok=%d\r\n",prescalerIdx);
			return prescalerIdx;
		}		
	}
	prescalerIdx=3;
	printf("prescalerIdx err,set default=%d\r\n",prescalerIdx);
	return prescalerIdx;
}

void Init_vk3224( void )
{
		/* 配置子串口1	*/
	VK32XX_VK32XX_UART_Base_InitTypeDef VK32XX_UART_InitStruct;
	VK32XX_VK32XX_UART_FIFO_InitTypeDef VK32XX_VK32XX_UART_FIFO_InitStruct;
	VK32XX_UART_InitStruct.UART_BaudRate = Init_vk3224_getBaudRateSet(115200);
  	VK32XX_UART_InitStruct.UART_WordLength = VK32XX_UART_WordLength_8b;
 	VK32XX_UART_InitStruct.UART_StopBits = VK32XX_UART_StopBits_1;
  	VK32XX_UART_InitStruct.UART_Parity = VK32XX_UART_Parity_0;
  	VK32XX_UART_InitStruct.UART_Mode = VK32XX_UART_Mode_RS232;
	VK32XX_UART_InitStruct.UART_EN=VK32XX_UART_ENABLE;
	VK32XX_UART_Base_Init( VKCOM1, &VK32XX_UART_InitStruct );
	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFTL = VK32XX_UART_TFTL_0BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFTL = VK32XX_UART_RFTL_1BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFCL = RESET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFCL = RESET;
	VK32XX_UART_FIFO_Init( VKCOM1, &VK32XX_VK32XX_UART_FIFO_InitStruct );
	VK32XX_UART_Clear_TXFIFO( VKCOM1 );
	VK32XX_UART_Clear_RXFIFO( VKCOM1 );
	VK32XX_UART_ITConfig( VKCOM1, VK32XX_UART_IT_RFIEN, SET );
	VK32XX_UART_Cmd( VKCOM1, SET );
	printf( "\r\n 配置串口1（波特率：115200；数据位：8位；无奇偶校验）。\r\n" );

			/* 配置子串口2	*/

	VK32XX_UART_InitStruct.UART_BaudRate = Init_vk3224_getBaudRateSet(115200);
  	VK32XX_UART_InitStruct.UART_WordLength = VK32XX_UART_WordLength_8b;
 	VK32XX_UART_InitStruct.UART_StopBits = VK32XX_UART_StopBits_1;
  	VK32XX_UART_InitStruct.UART_Parity = VK32XX_UART_Parity_0;
  	VK32XX_UART_InitStruct.UART_Mode = VK32XX_UART_Mode_RS232;
	VK32XX_UART_Base_Init( VKCOM2, &VK32XX_UART_InitStruct );

	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFTL = VK32XX_UART_TFTL_0BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFTL = VK32XX_UART_RFTL_1BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFCL = RESET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFCL = RESET;
	VK32XX_UART_FIFO_Init( VKCOM2, &VK32XX_VK32XX_UART_FIFO_InitStruct );

	VK32XX_UART_Clear_TXFIFO( VKCOM2 );

	VK32XX_UART_Clear_RXFIFO( VKCOM2 );

	VK32XX_UART_ITConfig( VKCOM2, VK32XX_UART_IT_RFIEN, SET );
	VK32XX_UART_Cmd( VKCOM2, SET );
	printf( "\r\n 配置串口2（波特率：115200；数据位：8位；无奇偶校验）。\r\n" );

				/* 配置子串口3	*/

	VK32XX_UART_InitStruct.UART_BaudRate = Init_vk3224_getBaudRateSet(115200);
  	VK32XX_UART_InitStruct.UART_WordLength = UART_WordLength_9b;
 	VK32XX_UART_InitStruct.UART_StopBits = VK32XX_UART_StopBits_1;
  	VK32XX_UART_InitStruct.UART_Parity = UART_Parity_Even;
  	VK32XX_UART_InitStruct.UART_Mode = VK32XX_UART_Mode_RS232;
	VK32XX_UART_Base_Init( VKCOM3, &VK32XX_UART_InitStruct );

	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFTL = VK32XX_UART_TFTL_0BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFTL = VK32XX_UART_RFTL_1BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFCL = RESET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFCL = RESET;
	VK32XX_UART_FIFO_Init( VKCOM3, &VK32XX_VK32XX_UART_FIFO_InitStruct );

	VK32XX_UART_Clear_TXFIFO( VKCOM3 );

	VK32XX_UART_Clear_RXFIFO( VKCOM3 );

	VK32XX_UART_ITConfig( VKCOM3, VK32XX_UART_IT_RFIEN, SET );
	VK32XX_UART_Cmd( VKCOM3, SET );
	printf( "\r\n 配置串口3（波特率：115200；数据位：8位；偶校验）。\r\n" );


		/* 配置子串口4	*/

	VK32XX_UART_InitStruct.UART_BaudRate = Init_vk3224_getBaudRateSet(115200);
  	VK32XX_UART_InitStruct.UART_WordLength = UART_WordLength_9b;
 	VK32XX_UART_InitStruct.UART_StopBits = VK32XX_UART_StopBits_1;
  	VK32XX_UART_InitStruct.UART_Parity = UART_Parity_Even;
  	VK32XX_UART_InitStruct.UART_Mode = VK32XX_UART_Mode_RS232;
	VK32XX_UART_Base_Init( VKCOM4, &VK32XX_UART_InitStruct );

	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFTL = VK32XX_UART_TFTL_0BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFTL = VK32XX_UART_RFTL_1BYTE	;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFEN = SET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_TFCL = RESET;
  	VK32XX_VK32XX_UART_FIFO_InitStruct.UART_RFCL = RESET;
	VK32XX_UART_FIFO_Init( VKCOM4, &VK32XX_VK32XX_UART_FIFO_InitStruct );

	VK32XX_UART_Clear_TXFIFO( VKCOM4 );

	VK32XX_UART_Clear_RXFIFO( VKCOM4 );

	VK32XX_UART_ITConfig( VKCOM4, VK32XX_UART_IT_RFIEN, SET );
	VK32XX_UART_Cmd( VKCOM4, SET );
	printf( "\r\n 配置串口4（波特率：115200；数据位：8位；偶校验）。\r\n" );




}
/********************************************************************************/
void debug_info(void)
{
	uint8_t temp,i=0;
	uint16_t data1=0,ch,addr,recv;
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==RESET)
		return;
	temp=USART_ReceiveData(USART1);
	USART_ClearFlag(USART1,USART_FLAG_RXNE);
	if(temp==0)
		return;
	printf("recv:%x\r\n",temp);
	switch(temp)
	{
		case '1':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SCTLR << 9;
			data1 = 0x8000|ch | addr;			
			data1|=0x18;
			printf("1-SCTLR:%x\r\n",data1);
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			VK32XX_SPI_Send_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			break;
		case '2':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SFOCR << 9;
			data1 = 0x8000|ch | addr;			
			data1|=0xff;
			printf("2-SFOCR:%x\r\n",data1);
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			VK32XX_SPI_Send_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();			
			break;
		case '3':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SFOCR << 9;
			data1 = 0x8000|ch | addr;			
			data1|=0x0c;
			printf("3-SFOCR:%x\r\n",data1);
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			VK32XX_SPI_Send_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();
			
			break;
		case '4':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SFSR << 9;
			data1 = ch | addr;			
			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("4-SFSR:%x,read:%x\r\n",data1,recv);
			break;
		case '5':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SCTLR << 9;
			data1 = ch | addr;			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("5-SCTLR:%x,read:%x\r\n",data1,recv);
			break;
		case '6':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SFOCR << 9;
			data1 = ch | addr;			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("6-SFOCR:%x,read:%x\r\n",data1,recv);
			
			break;
		case '7':		
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = GIR << 9;
			data1 = 0x8000|ch | addr;			
			data1|=0xf0;
			printf("7-GIR:%x\r\n",data1);
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			VK32XX_SPI_Send_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();				
			break;
		case '8':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = GIR << 9;
			data1 = ch | addr;			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("8-GIR:%x,read:%x\r\n",data1,recv);
			
			break;
		case '9':		
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SIER << 9;
			data1 = 0x8000|ch | addr;			
			data1|=0x01;
			printf("9-SIER:%x\r\n",data1);
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			VK32XX_SPI_Send_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();				
			break;
		case '0':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SIER << 9;
			data1 = ch | addr;			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("0-SIER:%x,read:%x\r\n",data1,recv);
			
			break;
		case 's':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SSR << 9;
			data1 = ch | addr;			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("s-SSR:%x,read:%x\r\n",data1,recv);		
			break;
		case 'a':		
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SCONR << 9;
			data1 = 0x8000|ch | addr;			
			data1|=0x10;
			printf("a-SCONR:%x\r\n",data1);
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			VK32XX_SPI_Send_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();				
			break;			
		case 'b':
			ch = VKCOM4;//0x0000 COM2=0X2000
			addr = SCONR << 9;
			data1 = ch | addr;			
			VK32XX_CS_LOW();	//使能VK3224的SPI功能
			recv=VK32XX_SPI_Receive_Data( data1 );
			VK32XX_delay(10);
			VK32XX_CS_HIGH();	
			printf("b-SCONR:%x,read:%x\r\n",data1,recv);
			
			break;			

		case 'x':
			printf("send:ab\r\n");
			VK32XX_UART_SendData(VKCOM4,0XAB);
			VK32XX_delay(10);
			break;		
		case 'y':			
			recv=VK32XX_UART_ReceiveData(VKCOM4);
			VK32XX_delay(10);
			printf("send:%x\r\n",recv);
			break;				
		default:
			break;
			
	}
	
	temp=0;
}


int main(void)
{
	 uint16_t i,j;
	 
	 uart_init();
	 //GPIO_Configuration ();	//GPIO端口配置函数

	 //NVIC_Configuration ();	//NVIC配置函数

	 SPI_Configuration ();	//SPI配置函数

	 VK32XX_EXTI_Configuration ();	//EXTI8配置函数	 
	 //EXTI_GenerateSWInterrupt(EXTI_Line8);

	 VK32XX_CS_HIGH(); 	//	禁止VK3224的SPI功能

	 printf( "\r\n ("__DATE__ " - " __TIME__ ") \r\n");
	 //VK32XX_delay( 100 );
	
	 //printf( "\r\n 正在测试与VK3224的SPI通信。\r\n" );
	 //VK32XX_delay( 100 );

	 //Init_VK3224_SPI( );

	 //printf( "\r\n 与VK3224的SPI通信测试成功。\r\n" );
	 //VK32XX_delay( 100 );

	printf( "\r\n 正在初始化VK3224。\r\n" );
	//VK32XX_delay( 100 );

	Init_vk3224( );

	printf( "\r\n 初始化VK3224完成。\r\n" );
	VK32XX_delay( 100 );	 
	 printf("dddd\n");
	 while(1)
	 {
		//for(i=0;i<20000;i++)
		//	for(j=0;j<100;j++);
		 //printf("send:ab\n");
		 //VK32XX_UART_SendData(VKCOM1,0X00AB);
		 debug_info();
	 }
}
