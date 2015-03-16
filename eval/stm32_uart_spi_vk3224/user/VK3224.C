#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_spi.h"
#include "VK3224.h"


uint16_t uartBaudRatePrescaler[16]=
{
	48,96,192,384,768,1536,3072,6144,16,32,64,128,256,512,1024,2048
};



/* 延时函数 ------------------------------------------------------------------*/

void VK32XX_delay ( uint16_t time )
{
	uint16_t a, b;
	for( a=0; a<=time; a++ )
		for( b=0; b<=3000; b++ );
} 
/********************************************************************************/




/**********************************
 * 函数名：VK32XX_SPI_Send_Data();
 * 描述  ：发送数据到VK3224
 * 输入  : uint16_t value
 * 输出  ：uint16_t rxd
 * 举例  ：无
 * 注意  ：无
*************************************/

uint16_t VK32XX_SPI_Send_Data( uint16_t value )
{
	uint16_t rxd;

	while( SPI_I2S_GetFlagStatus( VK32XX_SPI, SPI_I2S_FLAG_TXE ) == RESET );
	SPI_I2S_SendData( VK32XX_SPI, value );
	while( SPI_I2S_GetFlagStatus( VK32XX_SPI, SPI_I2S_FLAG_RXNE ) == RESET );
	rxd = SPI_I2S_ReceiveData( VK32XX_SPI );
	return rxd;
}
/********************************************************************************/




/**********************************
 * 函数名：VK32XX_SPI_Receive_Data();
 * 描述  ：从VK3224读取数据
 * 输入  : uint16_t value
 * 输出  ：uint16_t  rbuf
 * 举例  ：无
 * 注意  ：无
*************************************/

uint16_t VK32XX_SPI_Receive_Data( uint16_t value) 
{
	uint16_t rbuf;

	while( SPI_I2S_GetFlagStatus( VK32XX_SPI, SPI_I2S_FLAG_TXE ) == RESET );
	SPI_I2S_SendData( VK32XX_SPI, value );
	while( SPI_I2S_GetFlagStatus( VK32XX_SPI, SPI_I2S_FLAG_RXNE ) == RESET );
	rbuf = SPI_I2S_ReceiveData( VK32XX_SPI ) & 0x00ff;
	
	return rbuf;
}


/**********************************
 * 函数名：VK32XX_Data_Broadcast;
 * 描述  ：设置VK3224的广播功能
 * 输入  : uint16_t Function_State;
 * 输出  ：无
 * 举例  ：
 * 注意  ：无
*************************************/

void VK32XX_Data_Broadcast( uint16_t FunctionalState )
{
	uint16_t  addr, data1;
	uint16_t data;

	 /* Check the parameters */
	assert_param(IS_FUNCTION_STATE(FunctionalState));

	addr = GCR << 9;
	data1 = addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	if( FunctionalState	== SET ) 	data = data | 0x0080;
	else							data = data & 0x007f;

	data1 = 0x8000 | addr | data;

	VK32XX_SPI_Send_Data( data1 );
	VK32XX_CS_HIGH();
}
/********************************************************************************/





/**********************************
 * 函数名：VK32XX_Idel;
 * 描述  ：设置VK3224进入IDEL
 * 输入  : uint16_t Function_State;
 * 输出  ：无
 * 举例  ：
 * 注意  ：无
*************************************/

void VK32XX_Idel( uint16_t FunctionalState )
{
	uint16_t  addr, data1;
	uint16_t data;

	 /* Check the parameters */
	assert_param(IS_FUNCTION_STATE(FunctionalState));

	addr = GCR << 9;
	data1 = addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能
		
	data = VK32XX_SPI_Receive_Data( data1 );

	if( FunctionalState	== SET ) 	data = data | 0x0040;
	else							data = data & 0x00bf;

	data1 = 0x8000 | addr | data;

	VK32XX_SPI_Send_Data( data1 );
	VK32XX_CS_HIGH();
}
/********************************************************************************/




/**********************************
 * 函数名：VK32XX_UART_Base_Init;
 * 描述  ：初始化VK3224的UARTx的基本工作状态
 * 输入  : uint16_t UARTx；VK32XX_VK32XX_UART_Base_InitTypeDef* VK32XX_UART_InitStruct
 * 输出  ：无
 * 举例  ：VK32XX_UART_Base_Init( VKCOM1,  &UART1_InitStruct )
 * 注意  ：无
*************************************/

void VK32XX_UART_Base_Init( uint16_t UARTx, VK32XX_VK32XX_UART_Base_InitTypeDef* VK32XX_UART_InitStruct )
{
	uint16_t ch, data, addr, data1;
	printf("VK32XX_UART_Base_Init\n");
	 /* Check the parameters */
	assert_param(IS_UART_ALL_PERIPH(USARTx));
  	assert_param(IS_UART_BAUDRATE(VK32XX_UART_InitStruct->UART_BaudRate));  
  	assert_param(IS_UART_WORD_LENGTH(VK32XX_UART_InitStruct->UART_WordLength));
  	assert_param(IS_UART_STOPBITS(VK32XX_UART_InitStruct->UART_StopBits));
  	assert_param(IS_UART_PARITY(VK32XX_UART_InitStruct->UART_Parity));
  	assert_param(IS_UART_MODE(VK32XX_UART_InitStruct->UART_Mode));

	ch = UARTx;
	addr = SCTLR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = 0;//VK32XX_SPI_Receive_Data( data1 );

	//data = data & 0x000b;
	data = data | ((VK32XX_UART_InitStruct->UART_BaudRate)<<4) | (VK32XX_UART_InitStruct->UART_Mode)|(VK32XX_UART_InitStruct->UART_EN);
	data1 = ch | 0x8000 | addr |data; //sctlr.val;
	printf("SCTLR:%x\r\n",data1);
	VK32XX_SPI_Send_Data( data1 );
	
	VK32XX_delay(1);

	addr = SCONR << 9;
	data1 = ch | addr;

	data = 0;//data = VK32XX_SPI_Receive_Data( data1 );

	//data = data & 0x0007;
	data = data | (VK32XX_UART_InitStruct->UART_WordLength) | ( VK32XX_UART_InitStruct->UART_StopBits ) | ( VK32XX_UART_InitStruct->UART_Parity );
	data1 = ch | 0x8000 | addr | data;
	printf("SCONR:%x\r\n",data1);
	VK32XX_SPI_Send_Data( data1 );

	VK32XX_delay(1);

	VK32XX_CS_HIGH();
}
/********************************************************************************/

		


/**********************************
 * 函数名：VK32XX_UART_StructInit;
 * 描述  ：初始化VK3224的UARTx的结构体变量
 * 输入  : VK32XX_VK32XX_UART_Base_InitTypeDef* UART_InitStruc
 * 输出  ：无
 * 举例  ：VK32XX_UART_StructInit( &UART1_InitStruct )
 * 注意  ：无
*************************************/
void VK32XX_UART_StructInit( VK32XX_VK32XX_UART_Base_InitTypeDef* VK32XX_UART_InitStruct )
{
/* VK32XX_UART_InitStruct members default value */

  VK32XX_UART_InitStruct->UART_BaudRate = UART_BaudRate_16;
  VK32XX_UART_InitStruct->UART_WordLength = VK32XX_UART_WordLength_8b;
  VK32XX_UART_InitStruct->UART_StopBits = VK32XX_UART_StopBits_1;
  VK32XX_UART_InitStruct->UART_Parity = VK32XX_UART_Parity_0;
  VK32XX_UART_InitStruct->UART_Mode = VK32XX_UART_Mode_RS232;
}
/***********************************************************************************/




/**********************************
 * 函数名：VK32XX_UART_FIFO_Init;
 * 描述  ：初始化VK3224的UARTx关于FIFO基本工作状态
 * 输入  : uint16_t UARTx; VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct
 * 输出  ：无
 * 举例  ：VK32XX_UART_FIFO_Init( VKCOM1,  &UART1_FIFO_InitStruct )
 * 注意  ：无
*************************************/

void VK32XX_UART_FIFO_Init( uint16_t UARTx, VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct )
{
	uint16_t ch, data, addr, data1=0, data2=0;
	printf("VK32XX_UART_FIFO_Init\n");
	/* Check the parameters */
	assert_param(IS_UART_ALL_PERIPH(USARTx));
  	assert_param(IS_UART_TFTL(VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFTL));  
  	assert_param(IS_UART_RFTL(VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFTL));
  	assert_param(IS_FUNCTION_STATE(VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFEN));
  	assert_param(IS_FUNCTION_STATE(VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFEN));
  	assert_param(IS_FUNCTION_STATE(VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFCL));
	assert_param(IS_FUNCTION_STATE(VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFCL));		

	
	ch = UARTx;
	addr = SFOCR << 9;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能


	data1 = (VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFTL) | (VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFTL) | (VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFCL);

	data2 = (VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFEN) << 3;
	data1 = data1 | data2;

	data2 = (VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFEN) << 2;
	data1 = data1 | data2;

	data2 = (VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFCL) << 1;
	data1 = data1 | data2;
	 
	data = ch | 0x8000 | addr | data1;
	printf("SFOCR:%x\r\n",data);
	VK32XX_SPI_Send_Data( data );
	
	VK32XX_delay(1);

	VK32XX_CS_HIGH();
}
/***********************************************************************************/






/**********************************
 * 函数名：VK32XX_UART_FIFO_StructInit;
 * 描述  ：初始化VK3224的UARTx的FIFO结构体变量
 * 输入  : VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct
 * 输出  ：无
 * 举例  ：VK32XX_UART_FIFO_StructInit( &UART1_FIFO_InitStruct )
 * 注意  ：无
*************************************/

void VK32XX_UART_FIFO_StructInit( VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct )
{
/* VK32XX_VK32XX_UART_FIFO_InitStruct members default value */

  VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFTL = VK32XX_UART_TFTL_0BYTE	;
  VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFTL = VK32XX_UART_RFTL_1BYTE	;
  VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFEN = RESET;
  VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFEN = RESET;
  VK32XX_VK32XX_UART_FIFO_InitStruct->UART_TFCL = RESET;
  VK32XX_VK32XX_UART_FIFO_InitStruct->UART_RFCL = RESET;
}
/***********************************************************************************/




/**********************************
 * 函数名：VK32XX_UART_ITConfig;
 * 描述  ：配置VK3224的UARTx中断
 * 输入  :uint16_t UARTx, uint16_t UART_IT, uint16_t FunctionalState
 * 输出  ：无
 * 举例  ：VK32XX_UART_ITConfig( UART1, UART_IT_FOEIEN , SET )
 * 注意  ：无
*************************************/

void VK32XX_UART_ITConfig( uint16_t UARTx, uint16_t UART_IT, uint16_t FunctionalState )
{
	uint16_t ch, addr;
	uint16_t data1, data2;
	uint16_t IR;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
  	assert_param(IS_UART_CONFIG_IT(UART_IT));
  	assert_param(IS_FUNCTION_STATE(FunctionalState));

	ch = UARTx;
	addr = SIER << 9;
	data1 =  ch | addr ;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data2 = VK32XX_SPI_Receive_Data( data1 );
	IR = VK32XX_SPI_Receive_Data( 0x0600 );			//读GIR寄存器的值

	VK32XX_CS_HIGH();

/*	关闭UARTx的全局中断		*/

	if( UARTx == VKCOM1 ) 
	{
		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X00EF)) );

		VK32XX_CS_HIGH();
	}
	else if( UARTx == VKCOM2 ) 
	{
		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X00dF)) );

		VK32XX_CS_HIGH();
	}
	else if( UARTx == VKCOM3 ) 
	{
		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X00bF)) );

		VK32XX_CS_HIGH();
	}
	else
	{
		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X007F)) );

		VK32XX_CS_HIGH();
	}
				

	if( FunctionalState == SET )
	{
		data1 = data2 | UART_IT;
		data1 = 0x8000 | ch | addr | data1 ;

		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();

	/*	使能UARTx的全局中断		*/

		if( UARTx == VKCOM1 ) 
		{
			VK32XX_CS_LOW();	//使能VK3224的SPI功能

			VK32XX_SPI_Send_Data( (0x8600 | (IR | 0X0010)) );

			VK32XX_CS_HIGH();
		}
		else if( UARTx == VKCOM2 ) 
		{
			VK32XX_CS_LOW();	//使能VK3224的SPI功能

			VK32XX_SPI_Send_Data( (0x8600 | (IR|0X0020)) );

			VK32XX_CS_HIGH();
		}
		else if( UARTx == VKCOM3 ) 
		{
			VK32XX_CS_LOW();	//使能VK3224的SPI功能

			VK32XX_SPI_Send_Data( (0x8600 | (IR|0X0040)) );

			VK32XX_CS_HIGH();
		}
		else
		{
			VK32XX_CS_LOW();	//使能VK3224的SPI功能

			VK32XX_SPI_Send_Data( (0x8600 | (IR|0X0080)) );

			VK32XX_CS_HIGH();
		}

	}
	else
	{
		if( UART_IT == UART_IT_FOEIEN ) 		data1 = data2 & 0x00bf;
		else if( UART_IT == UART_IT_TRIEN )		data1 = data2 & 0x00fd;
		else 									data1 = data2 & 0x00fe;

		data1 = data1 | 0x8000 | ch | addr ;

		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();
	}
}
/***********************************************************************************/




/**********************************
 * 函数名：VK32XX_UART_Cmd;
 * 描述  ：Enables or disables the specified UART peripheral
 * 输入  :uint16_t UARTx, uint16_t UART_IT, uint16_t FunctionalState
 * 输出  ：无
 * 举例  ：VK32XX_UART_Cmd( VKCOM1, SET )
 * 注意  ：无
*************************************/

void VK32XX_UART_Cmd( uint16_t UARTx, uint16_t FunctionalState )
{
	uint16_t ch, data1, data2, addr;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
  	assert_param(IS_FUNCTION_STATE(FunctionalState));

  	ch = UARTx;
  	addr = SCTLR << 9;
	data1 = ch | addr;
  
  	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data2 = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH(); 

	if(  FunctionalState == SET )
	{
		data1 = 0x8000 | ch | addr | ( data2 | 0x0008);

		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();
	}
	else
	{
		data1 = 0x8000 | ch | addr | ( data2 & 0x00F7);	

		VK32XX_CS_LOW();	//使能VK3224的SPI功能

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();
	}
}
/***********************************************************************************/




/**********************************
 * 函数名：VK32XX_UART_SendData;
 * 描述  ： Transmits single data through the UARTx peripheral.
 * 输入  :uint16_t UARTx,  uint16_t data
 * 输出  ：无
 * 举例  ：VK32XX_UART_SendData( VKCOM1, 0xffff )
 * 注意  ：
*************************************/

void VK32XX_UART_SendData( uint16_t UARTx, uint16_t data )
{
	uint16_t ch, data1, addr;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
	
	ch = UARTx;
	addr = SFDR << 9;
	data1 = 0x8000 | ch | addr | data ;
	printf("SFDR:%x\r\n",data1);
	VK32XX_CS_LOW();	//使能VK3224的SPI功能
	
	VK32XX_SPI_Send_Data( data1 );
	VK32XX_delay(10);
	VK32XX_CS_HIGH();
}
/***********************************************************************************/





/**********************************
 * 函数名：VK32XX_UART_ReceiveData;
 * 描述  ： Returns the most recent received data by the UARTx peripheral.
 * 输入  :uint16_t UARTx
 * 输出  ：读UARTx的FIFO寄存器的数值
 * 举例  ：VK32XX_UART_ReceiveData( VKCOM1 )
 * 注意  ：
*************************************/

uint16_t VK32XX_UART_ReceiveData( uint16_t UARTx )
{
	uint16_t ch, addr, data1;
	uint16_t data;
	 
  /* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
  
  /* Receive Data */

	ch =  UARTx;
	addr = SFDR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH(); 

	printf("recv data=%x\r\n",data);
  	return ( data );
}
/***********************************************************************************/




/**********************************
 * 函数名：VK32XX_Get_IT_UARTx;
 * 描述  :获取子串口中断号
 * 输入  :无
 * 输出  ：返回子串口号
 * 举例  ：
 * 注意  ：
*************************************/

uint16_t VK32XX_Get_IT_UARTx( void )
{
	uint16_t addr, data1;
	uint16_t data;

	addr = GIR << 9;
	data1 = addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();
	
	data = data & 0x000f;

	if( (data & 0x0008) == 0x0008 )				return VKCOM4;
    else if( (data & 0x0004) == 0x0004 )		return VKCOM3;
	else if( (data & 0x0002) == 0x0002 )		return VKCOM2;
	else										return VKCOM1;
}
/***********************************************************************************/	 






/**********************************
 * 函数名：VK32XX_UART_GetFlagStatus;
 * 描述  :查询子串口的各种状态
 * 输入  :uint16_t UARTx, uint8_t UART_FLAG
 * 输出  ：
 * 举例  ：VK32XX_UART_GetFlagStatus( VKCOM1, UART_FLAG_OE);
 * 注意  ：
*************************************/

uint16_t VK32XX_UART_GetFlagStatus( uint16_t UARTx, uint16_t UART_FLAG )
{
	uint16_t ch, addr, data1;
	uint16_t data;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
  	assert_param(IS_UART_FLAG(UART_FLAG));

  	ch = UARTx;
  	addr = SSR << 9;
  	data1 = ch | addr;

  	VK32XX_CS_LOW();	//使能VK3224的SPI功能
	printf("read SSR:%x\r\n",data1);
	data = VK32XX_SPI_Receive_Data( data1 );
	printf("read:%x\r\n",data);
	VK32XX_CS_HIGH(); 

	if( (data & UART_FLAG) == UART_FLAG ) 	return SET;
	else 									return RESET;
}
/***********************************************************************************/




/**********************************
 * 函数名：VK32XX_UART_GetITStatus;
 * 描述  :查询子串口的中断状态
 * 输入  :uint16_t UARTx, uint16_t UART_IT
 * 输出  ：
 * 举例  ：VK32XX_UART_GetITStatus( VKCOM1, UART_IT_FOEIEN );
 * 注意  ：
*************************************/

uint16_t VK32XX_UART_GetITStatus( uint16_t UARTx, uint16_t UART_IT)
{
	uint16_t ch, addr, data1;
	uint16_t data;
	
	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
 	assert_param(IS_UART_GET_IT(UART_IT));

	ch = UARTx;
	addr = SIFR << 9;
	data1 = ch | addr;
	
	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();

	if( (data & UART_IT)  == UART_IT ) 	return 	SET;
	else								return  RESET;
}
/***********************************************************************************/




	
/**********************************
 * 函数名：VK32XX_UART_ClearITPendingBit;
 * 描述  :清楚子串口的中断标志位
 * 输入  :uint16_t UARTx, uint16_t UART_IT
 * 输出  ：
 * 举例  ：VK32XX_UART_ClearITPendingBit( VKCOM1, UART_IT_FOEIEN );
 * 注意  ：
*************************************/

void VK32XX_UART_ClearITPendingBit( uint16_t UARTx, uint16_t UART_IT )
{
	
	uint16_t ch, addr, data1;
	uint16_t data, IFR;
	
	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
  	assert_param(IS_UART_CLEAR_IT(UART_IT));
	
	ch = UARTx;
	addr = SIFR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	IFR = VK32XX_SPI_Receive_Data( data1 );

	if(  UART_IT == UART_IT_FOEIEN  )  		data = IFR & 0X00BF;
	else if( UART_IT == UART_IT_TRIEN )  	data = IFR & 0X00fd;
	else									data = IFR & 0X00fe;
	
	data1 = data1 | 0x8000 | data;

	VK32XX_SPI_Send_Data( data1 );

	VK32XX_CS_HIGH();
}
/***********************************************************************************/





/**********************************
 * 函数名： VK32XX_UART_Get_Num_TXFIFO;
 * 描述  : 查询发送FIFO的字节数
 * 输入  : uint16_t UARTx
 * 输出  ：uint8_t num;
 * 举例  ：VK32XX_UART_Get_Num_TXFIFO( VKCOM1 )
 * 注意  ：
*************************************/

uint8_t VK32XX_UART_Get_Num_TXFIFO( uint16_t UARTx )
{
	uint16_t ch, addr, data1, data;
	uint8_t num;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFSR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();

	data = data >> 4;

	num = ( (uint8_t) data );
	return num;
}
/***********************************************************************************/





/**********************************
 * 函数名： VK32XX_UART_Get_Num_RXFIFO;
 * 描述  : 查询接收FIFO的字节数
 * 输入  : uint16_t UARTx
 * 输出  ：uint8_t num;
 * 举例  ：VK32XX_UART_Get_Num_RXFIFO( VKCOM1 )
 * 注意  ：
*************************************/

uint8_t VK32XX_UART_Get_Num_RXFIFO( uint16_t UARTx )
{
	uint16_t ch, addr, data1, data;
	uint8_t num;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFSR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();

	data = data & 0X000F;

	num = ( (uint8_t) data );
	return num;
}
/***********************************************************************************/




/**********************************
 * 函数名： VK32XX_UART_Clear_TXFIFO
 * 描述  : 清空发送FIFO
 * 输入  : uint16_t UARTx
 * 输出  ：无
 * 举例  ：VK32XX_UART_Clear_TXFIFO( VKCOM1 )
 * 注意  ：
*************************************/

void VK32XX_UART_Clear_TXFIFO( uint16_t UARTx )
{
	uint16_t ch, addr, data1, data,flag=0;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));
	//do{
	ch = UARTx;
	addr = SFOCR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能
	//data1=0x92ff;
	//printf("SFOCR:%x\r\n",data1);
	//VK32XX_SPI_Send_Data(data1);//test
	data = VK32XX_SPI_Receive_Data( data1 );

	data = data | 0x0002;

	data1 = 0x8000 | ch | addr | data;
	printf("SFOCR:%x\r\n",data1);
	VK32XX_SPI_Send_Data( data1 );
	VK32XX_delay(10);
	VK32XX_CS_HIGH();
	//if(VK32XX_UART_GetFlagStatus(  UARTx, UART_FLAG_TFEM ) == SET)
	//	flag=1;
	//}
	//while(flag==0);
	while(VK32XX_UART_GetFlagStatus(  UARTx, UART_FLAG_TFEM ) == RESET);

	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	data = data & 0x00fd;

	data1 = 0x8000 | ch | addr | data;

	VK32XX_SPI_Send_Data( data1 );
	VK32XX_delay(10);
	VK32XX_CS_HIGH();
}
/***********************************************************************************/


	


/**********************************
 * 函数名： VK32XX_UART_Clear_RXFIFO
 * 描述  : 清空接收FIFO
 * 输入  : uint16_t UARTx
 * 输出  ：无
 * 举例  ：VK32XX_UART_Clear_RXFIFO( VKCOM1 )
 * 注意  ：
*************************************/

void VK32XX_UART_Clear_RXFIFO( uint16_t UARTx )
{
	uint16_t ch, addr, data1, data;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFOCR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	data = data | 0x0001;

	data1 = 0x8000 | ch | addr | data;
	printf("SFOCR:%x\r\n",data1);
	VK32XX_SPI_Send_Data( data1 );

	VK32XX_CS_HIGH();

	while( VK32XX_UART_GetFlagStatus(  UARTx, UART_FLAG_RFEM ) == RESET );

	data1 = ch | addr;

	VK32XX_CS_LOW();	//使能VK3224的SPI功能

	data = VK32XX_SPI_Receive_Data( data1 );

	data = data & 0x00fE;

	data1 = 0x8000 | ch | addr | data;

	VK32XX_SPI_Send_Data( data1 );

	VK32XX_CS_HIGH();
}
/***********************************************************************************/










