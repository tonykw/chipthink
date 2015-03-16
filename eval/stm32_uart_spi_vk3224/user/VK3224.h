#ifndef _VK3224_H
#define _VK3224_H
#endif

#include "stm32f2xx.h"
#define vk32xx_fr 11059200


 

/* M25P FLASH SPI Interface pins  */  
#define VK32XX_SPI                           SPI1
#define VK32XX_SPI_CLK                       RCC_APB2Periph_SPI1
#define VK32XX_SPI_CLK_INIT                  RCC_APB2PeriphClockCmd

#define VK32XX_SPI_SCK_PIN                   GPIO_Pin_5
#define VK32XX_SPI_SCK_GPIO_PORT             GPIOA
#define VK32XX_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define VK32XX_SPI_SCK_SOURCE                GPIO_PinSource5
#define VK32XX_SPI_SCK_AF                    GPIO_AF_SPI1

#define VK32XX_SPI_MISO_PIN                  GPIO_Pin_6
#define VK32XX_SPI_MISO_GPIO_PORT            GPIOA
#define VK32XX_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define VK32XX_SPI_MISO_SOURCE               GPIO_PinSource6
#define VK32XX_SPI_MISO_AF                   GPIO_AF_SPI1

#define VK32XX_SPI_MOSI_PIN                  GPIO_Pin_5
#define VK32XX_SPI_MOSI_GPIO_PORT            GPIOB
#define VK32XX_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define VK32XX_SPI_MOSI_SOURCE               GPIO_PinSource5
#define VK32XX_SPI_MOSI_AF                   GPIO_AF_SPI1

#define VK32XX_CS_PIN                        GPIO_Pin_4
#define VK32XX_CS_GPIO_PORT                  GPIOA
#define VK32XX_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOA

#define VK32XX_IRQ_PIN                        GPIO_Pin_8
#define VK32XX_IRQ_GPIO_PORT                  GPIOB
#define VK32XX_IRQ_GPIO_CLK                   RCC_AHB1Periph_GPIOB
#define VK32XX_IRQ_PORT_SOURCE                EXTI_PortSourceGPIOB
#define VK32XX_IRQ_PIN_SOURCE                 EXTI_PinSource8
#define VK32XX_IRQ_EXTI_LINE                  EXTI_Line8




/* Exported macro ------------------------------------------------------------*/
/* Select VK32XX: Chip Select pin low */
#define VK32XX_CS_LOW()       GPIO_ResetBits(VK32XX_CS_GPIO_PORT, VK32XX_CS_PIN)
/* Deselect VK32XX: Chip Select pin high */
#define VK32XX_CS_HIGH()      GPIO_SetBits(VK32XX_CS_GPIO_PORT, VK32XX_CS_PIN)  


/*		全局寄存器列表		*/
#define GCR			((uint16_t)0x0001)	//全局控制寄存器
#define GIR			((uint16_t)0x0003)	//全局中断寄存器


/*		子串口寄存器列表		*/
#define SCTLR		((uint16_t)0x0006)	//子串口控制寄存器 
#define SCONR		((uint16_t)0x0007)	//子串口配置寄存器 
#define SFOCR		((uint16_t)0x0009)	//子串口FIFO控制寄存器 
#define	SIER		((uint16_t)0x000B)	//子串口中断使能寄存器 
#define	SIFR		((uint16_t)0x000C)	//子串口中断标志寄存器 
#define	SSR			((uint16_t)0x000D)	//子串口状态寄存器 
#define	SFSR		((uint16_t)0x000E)	//子串口FIFO状态寄存器 
#define	SFDR		((uint16_t)0x000F)	//子串口FIFO数据寄存器


typedef struct
{
  uint16_t UART_BaudRate;            /*!< This member configures the USART communication baud rate. */
                                       

  uint16_t UART_WordLength;          /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */

  uint16_t UART_StopBits;            /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */
  uint16_t UART_Parity;   			  /*!< Specifies the parity mode.*/

  uint16_t UART_Mode;
  uint16_t UART_EN;

} VK32XX_VK32XX_UART_Base_InitTypeDef;


typedef struct
{
  uint16_t UART_TFTL;  //发送FIFO触点控制       
                                       
  uint16_t UART_RFTL;  //接收FIFO触点控制     
                                        

  uint16_t UART_TFEN;  //发送FIFO使能控制位        
                                        
  uint16_t UART_RFEN;  //接收FIFO使能			

  uint16_t UART_TFCL;  //清除发送FIFO

  uint16_t UART_RFCL;  // 清除接收FIFO 

} VK32XX_VK32XX_UART_FIFO_InitTypeDef;



#define VK32XX_UART_ENABLE 								((uint16_t)0x0008)
#define UART_DISABLE 								((uint16_t)0x0000)


#define SET 								((uint16_t)0x0001)
#define RESET 								((uint16_t)0x0000)
#define IS_FUNCTION_STATE(STATE) (((STATE) == SET) || \
                                     ((STATE) == RESET) )


#define VKCOM1 								((uint16_t)0x0000)
#define VKCOM2								((uint16_t)0x2000)
#define VKCOM3 								((uint16_t)0x4000)
#define VKCOM4 								((uint16_t)0x6000)
#define IS_UART_ALL_PERIPH(COM)			(((COM) == VKCOM1) 	|| \
                                 			((COM) == VKCOM2) 	|| \
                                 			((COM) == VKCOM3)	|| \
								 			((COM) == VKCOM4))



#define UART_BaudRate_3              		((uint16_t)0x0000)
#define UART_BaudRate_6                  	 ((uint16_t)0x0010)
#define UART_BaudRate_12                	((uint16_t)0x0020)
#define UART_BaudRate_24                  	 ((uint16_t)0x0030)
#define UART_BaudRate_48                	((uint16_t)0x0040)
#define UART_BaudRate_96                 	 ((uint16_t)0x0050)
#define UART_BaudRate_192                	((uint16_t)0x0060)
#define UART_BaudRate_384                  	 ((uint16_t)0x0070)
#define UART_BaudRate_1               		 ((uint16_t)0x0080)
#define UART_BaudRate_2                 	 ((uint16_t)0x0090)
#define UART_BaudRate_4                		((uint16_t)0x00a0)
#define UART_BaudRate_8                	 	((uint16_t)0x00b0)
#define UART_BaudRate_16                	 ((uint16_t)0x00c0)
#define UART_BaudRate_32                	 ((uint16_t)0x00d0)
#define UART_BaudRate_64                	 ((uint16_t)0x00e0)
#define UART_BaudRate_128                	 ((uint16_t)0x00f0)
#define IS_UART_BAUDRATE(BAUDRATE)			 (((BAUDRATE) == UART_BaudRate_3  ) || \
                                 			((BAUDRATE) == UART_BaudRate_6 ) || \
                                 			((BAUDRATE) == UART_BaudRate_12 )    || \
								 			((BAUDRATE) == UART_BaudRate_24	)   || \
								 			((BAUDRATE) == UART_BaudRate_48	)   || \
											((BAUDRATE) == UART_BaudRate_96	)   || \
											((BAUDRATE) == UART_BaudRate_192)	|| \
											((BAUDRATE) == UART_BaudRate_384)	|| \
											((BAUDRATE) == UART_BaudRate_1	)   || \
											((BAUDRATE) == UART_BaudRate_2	)	|| \
											((BAUDRATE) == UART_BaudRate_4	)	|| \
											((BAUDRATE) == UART_BaudRate_8	)   || \
											((BAUDRATE) == UART_BaudRate_16	)	|| \
											((BAUDRATE) == UART_BaudRate_32	)	|| \
											((BAUDRATE) == UART_BaudRate_64 )   || \
											((BAUDRATE) == UART_BaudRate_128))





#define VK32XX_UART_WordLength_8b                  ((uint16_t)0x0000)
#define UART_WordLength_9b                  ((uint16_t)0x0040)                                  
#define IS_UART_WORD_LENGTH(LENGTH) (((LENGTH) == VK32XX_UART_WordLength_8b) || \
                                      ((LENGTH) == UART_WordLength_9b))


#define VK32XX_UART_StopBits_1                     ((uint16_t)0x0000)
#define UART_StopBits_2                     ((uint16_t)0x0080)
#define IS_UART_STOPBITS(STOPBITS) (((STOPBITS) == VK32XX_UART_StopBits_1) || \
                                     ((STOPBITS) == UART_StopBits_2) )


#define VK32XX_UART_Parity_0                     ((uint16_t)0x0000)
#define UART_Parity_Even                    ((uint16_t)0x0010)
#define UART_Parity_Odd                     ((uint16_t)0x0008)
#define UART_Parity_1                     ((uint16_t)0x0018)
#define IS_UART_PARITY(PARITY) (((PARITY) == VK32XX_UART_Parity_0) || \
                                 ((PARITY) == UART_Parity_Even) || \
                                 ((PARITY) == UART_Parity_Odd)	 || \
								 ((PARITY) == UART_Parity_1)	)


#define VK32XX_UART_Mode_RS232                        ((uint16_t)0x0000)
#define UART_Mode_NONE                       	((uint16_t)0x0004)
#define IS_UART_MODE(MODE) 			(((MODE) == VK32XX_UART_Mode_RS232) || \
                                     ((MODE) == UART_Mode_NONE) ||)


#define VK32XX_UART_TFTL_0BYTE							((uint16_t)0x0000)
#define UART_TFTL_4BYTE							((uint16_t)0x0040)
#define UART_TFTL_8BYTE							((uint16_t)0x0080)
#define UART_TFTL_12BYTE						((uint16_t)0x00C0)
#define IS_UART_TFTL(TFTL) 		(((TFTL) == VK32XX_UART_TFTL_0BYTE	) || \
                                 ((TFTL) == UART_TFTL_4BYTE	) || \
                                 ((TFTL) == UART_TFTL_8BYTE	)	 || \
								 ((TFTL) == UART_TFTL_12BYTE)	)



#define VK32XX_UART_RFTL_1BYTE							((uint16_t)0x0000)
#define UART_RFTL_4BYTE							((uint16_t)0x0010)
#define UART_RFTL_8BYTE							((uint16_t)0x0020)
#define UART_RFTL_14BYTE						((uint16_t)0x0030)
#define IS_UART_RFTL(RFTL) 		(((RFTL) == UART_RFTL_0BYTE	) || \
                                 ((RFTL) == UART_RFTL_4BYTE	) || \
                                 ((RFTL) == UART_RFTL_8BYTE	)	 || \
								 ((RFTL) == UART_RFTL_12BYTE)	)



#define UART_IT_FOEIEN                      ((uint16_t)0x0040)		//FIFO数据错误中断使能位
#define UART_IT_TRIEN                        ((uint16_t)0x0002)		//发送FIFO触点中断使能位
#define VK32XX_UART_IT_RFIEN                        ((uint16_t)0x0001)		//使能接收FIFO触点中断 
#define IS_UART_CONFIG_IT(IT) 	(((IT) == UART_IT_FOEIEN ) || \
								 ((IT) == UART_IT_TRIEN	) || \
								 ((IT) == VK32XX_UART_IT_RFIEN	) )


#define IS_UART_GET_IT(IT)		(((IT) == UART_IT_FOEIEN ) || \
								 ((IT) == UART_IT_TRIEN	) || \
								 ((IT) == VK32XX_UART_IT_RFIEN	) )


#define IS_UART_CLEAR_IT(IT)	(((IT) == UART_IT_FOEIEN ) || \
								 ((IT) == UART_IT_TRIEN	) || \
								 ((IT) == VK32XX_UART_IT_RFIEN	) )


#define UART_FLAG_OE                       ((uint16_t)0x0080)		//子串口接收 FIFO 中当前数据(最早写入)的溢出错误标志位
#define UART_FLAG_FE                       ((uint16_t)0x0040)		//子串口接收 FIFO 中当前数据(最早写入)的帧错误标志位
#define UART_FLAG_PE                       ((uint16_t)0x0020)		//子串口接收 FIFO 中当前数据(最早写入)的校验错误标志位
#define UART_FLAG_TFFL                     ((uint16_t)0x0008)	   	//子串口发送 FIFO 满标志
#define UART_FLAG_TFEM                     ((uint16_t)0x0004)		//子串口发送 FIFO 空标志
#define UART_FLAG_TXBY                     ((uint16_t)0x0002)		//子串口发送 TX 忙标志
#define UART_FLAG_RFEM                      ((uint16_t)0x0001)		//子串口接收 FIFO 空标志
#define IS_UART_FLAG(FLAG) (((FLAG) == UART_FLAG_OE) || ((FLAG) == UART_FLAG_FE) || \
                             ((FLAG) ==  UART_FLAG_PE) || ((FLAG) == UART_FLAG_TFFL) || \
                             ((FLAG) == UART_FLAG_TFEM ) || ((FLAG) == UART_FLAG_TXBY) || \
                             ((FLAG) ==  UART_FLAG_RFEM) )








void VK32XX_delay ( uint16_t time );

uint16_t VK32XX_SPI_Send_Data( uint16_t value );

uint16_t VK32XX_SPI_Receive_Data( uint16_t value);

void Init_VK3224_SPI( void );

void VK32XX_Data_Broadcast( uint16_t FunctionalState );		//设置VK3224的广播功能

void VK32XX_Idel( uint16_t FunctionalState );				//设置VK3224进入IDEL

void VK32XX_UART_Base_Init( uint16_t UARTx, VK32XX_VK32XX_UART_Base_InitTypeDef* VK32XX_UART_InitStruct );		//初始化VK3224的UARTx的基本工作状态

void VK32XX_UART_StructInit( VK32XX_VK32XX_UART_Base_InitTypeDef* VK32XX_UART_InitStruct );

void VK32XX_UART_FIFO_Init( uint16_t UARTx, VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct );			//初始化VK3224的UARTx关于FIFO基本工作状态

void VK32XX_UART_FIFO_StructInit( VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct );

void VK32XX_UART_ITConfig( uint16_t UARTx, uint16_t UART_IT, uint16_t FunctionalState );			//配置VK3224的UARTx中断

void VK32XX_UART_Cmd( uint16_t UARTx, uint16_t FunctionalState );		//Enables or disables the specified UART peripheral

void VK32XX_UART_SendData( uint16_t UARTx, uint16_t data );			//Transmits single data through the UARTx peripheral

uint16_t VK32XX_UART_ReceiveData( uint16_t UARTx );			//Returns the most recent received data by the UARTx peripheral

uint16_t VK32XX_Get_IT_UARTx( void );						//获取子串口中断号

uint16_t VK32XX_UART_GetFlagStatus( uint16_t UARTx, uint16_t UART_FLAG );		//查询子串口的各种状态

uint16_t VK32XX_UART_GetITStatus( uint16_t UARTx, uint16_t UART_IT);			//查询子串口的中断状态

void VK32XX_UART_ClearITPendingBit( uint16_t UARTx, uint16_t UART_IT );		//清楚子串口的中断标志位

uint8_t VK32XX_UART_Get_Num_TXFIFO( uint16_t UARTx );							//查询发送FIFO的字节数

uint8_t VK32XX_UART_Get_Num_RXFIFO( uint16_t UARTx );							// 查询接收FIFO的字节数

void VK32XX_UART_Clear_TXFIFO( uint16_t UARTx );								//清空发送FIFO

void VK32XX_UART_Clear_RXFIFO( uint16_t UARTx );								//清空接收FIFO









 

