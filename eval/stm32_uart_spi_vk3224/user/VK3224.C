#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_spi.h"
#include "VK3224.h"


uint16_t uartBaudRatePrescaler[16]=
{
	48,96,192,384,768,1536,3072,6144,16,32,64,128,256,512,1024,2048
};



/* ��ʱ���� ------------------------------------------------------------------*/

void VK32XX_delay ( uint16_t time )
{
	uint16_t a, b;
	for( a=0; a<=time; a++ )
		for( b=0; b<=3000; b++ );
} 
/********************************************************************************/




/**********************************
 * ��������VK32XX_SPI_Send_Data();
 * ����  ���������ݵ�VK3224
 * ����  : uint16_t value
 * ���  ��uint16_t rxd
 * ����  ����
 * ע��  ����
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
 * ��������VK32XX_SPI_Receive_Data();
 * ����  ����VK3224��ȡ����
 * ����  : uint16_t value
 * ���  ��uint16_t  rbuf
 * ����  ����
 * ע��  ����
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
 * ��������VK32XX_Data_Broadcast;
 * ����  ������VK3224�Ĺ㲥����
 * ����  : uint16_t Function_State;
 * ���  ����
 * ����  ��
 * ע��  ����
*************************************/

void VK32XX_Data_Broadcast( uint16_t FunctionalState )
{
	uint16_t  addr, data1;
	uint16_t data;

	 /* Check the parameters */
	assert_param(IS_FUNCTION_STATE(FunctionalState));

	addr = GCR << 9;
	data1 = addr;

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	if( FunctionalState	== SET ) 	data = data | 0x0080;
	else							data = data & 0x007f;

	data1 = 0x8000 | addr | data;

	VK32XX_SPI_Send_Data( data1 );
	VK32XX_CS_HIGH();
}
/********************************************************************************/





/**********************************
 * ��������VK32XX_Idel;
 * ����  ������VK3224����IDEL
 * ����  : uint16_t Function_State;
 * ���  ����
 * ����  ��
 * ע��  ����
*************************************/

void VK32XX_Idel( uint16_t FunctionalState )
{
	uint16_t  addr, data1;
	uint16_t data;

	 /* Check the parameters */
	assert_param(IS_FUNCTION_STATE(FunctionalState));

	addr = GCR << 9;
	data1 = addr;

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����
		
	data = VK32XX_SPI_Receive_Data( data1 );

	if( FunctionalState	== SET ) 	data = data | 0x0040;
	else							data = data & 0x00bf;

	data1 = 0x8000 | addr | data;

	VK32XX_SPI_Send_Data( data1 );
	VK32XX_CS_HIGH();
}
/********************************************************************************/




/**********************************
 * ��������VK32XX_UART_Base_Init;
 * ����  ����ʼ��VK3224��UARTx�Ļ�������״̬
 * ����  : uint16_t UARTx��VK32XX_VK32XX_UART_Base_InitTypeDef* VK32XX_UART_InitStruct
 * ���  ����
 * ����  ��VK32XX_UART_Base_Init( VKCOM1,  &UART1_InitStruct )
 * ע��  ����
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

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
 * ��������VK32XX_UART_StructInit;
 * ����  ����ʼ��VK3224��UARTx�Ľṹ�����
 * ����  : VK32XX_VK32XX_UART_Base_InitTypeDef* UART_InitStruc
 * ���  ����
 * ����  ��VK32XX_UART_StructInit( &UART1_InitStruct )
 * ע��  ����
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
 * ��������VK32XX_UART_FIFO_Init;
 * ����  ����ʼ��VK3224��UARTx����FIFO��������״̬
 * ����  : uint16_t UARTx; VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct
 * ���  ����
 * ����  ��VK32XX_UART_FIFO_Init( VKCOM1,  &UART1_FIFO_InitStruct )
 * ע��  ����
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����


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
 * ��������VK32XX_UART_FIFO_StructInit;
 * ����  ����ʼ��VK3224��UARTx��FIFO�ṹ�����
 * ����  : VK32XX_VK32XX_UART_FIFO_InitTypeDef* VK32XX_VK32XX_UART_FIFO_InitStruct
 * ���  ����
 * ����  ��VK32XX_UART_FIFO_StructInit( &UART1_FIFO_InitStruct )
 * ע��  ����
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
 * ��������VK32XX_UART_ITConfig;
 * ����  ������VK3224��UARTx�ж�
 * ����  :uint16_t UARTx, uint16_t UART_IT, uint16_t FunctionalState
 * ���  ����
 * ����  ��VK32XX_UART_ITConfig( UART1, UART_IT_FOEIEN , SET )
 * ע��  ����
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data2 = VK32XX_SPI_Receive_Data( data1 );
	IR = VK32XX_SPI_Receive_Data( 0x0600 );			//��GIR�Ĵ�����ֵ

	VK32XX_CS_HIGH();

/*	�ر�UARTx��ȫ���ж�		*/

	if( UARTx == VKCOM1 ) 
	{
		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X00EF)) );

		VK32XX_CS_HIGH();
	}
	else if( UARTx == VKCOM2 ) 
	{
		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X00dF)) );

		VK32XX_CS_HIGH();
	}
	else if( UARTx == VKCOM3 ) 
	{
		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X00bF)) );

		VK32XX_CS_HIGH();
	}
	else
	{
		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( (0x8600 | (IR&0X007F)) );

		VK32XX_CS_HIGH();
	}
				

	if( FunctionalState == SET )
	{
		data1 = data2 | UART_IT;
		data1 = 0x8000 | ch | addr | data1 ;

		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();

	/*	ʹ��UARTx��ȫ���ж�		*/

		if( UARTx == VKCOM1 ) 
		{
			VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

			VK32XX_SPI_Send_Data( (0x8600 | (IR | 0X0010)) );

			VK32XX_CS_HIGH();
		}
		else if( UARTx == VKCOM2 ) 
		{
			VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

			VK32XX_SPI_Send_Data( (0x8600 | (IR|0X0020)) );

			VK32XX_CS_HIGH();
		}
		else if( UARTx == VKCOM3 ) 
		{
			VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

			VK32XX_SPI_Send_Data( (0x8600 | (IR|0X0040)) );

			VK32XX_CS_HIGH();
		}
		else
		{
			VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

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

		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();
	}
}
/***********************************************************************************/




/**********************************
 * ��������VK32XX_UART_Cmd;
 * ����  ��Enables or disables the specified UART peripheral
 * ����  :uint16_t UARTx, uint16_t UART_IT, uint16_t FunctionalState
 * ���  ����
 * ����  ��VK32XX_UART_Cmd( VKCOM1, SET )
 * ע��  ����
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
  
  	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data2 = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH(); 

	if(  FunctionalState == SET )
	{
		data1 = 0x8000 | ch | addr | ( data2 | 0x0008);

		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();
	}
	else
	{
		data1 = 0x8000 | ch | addr | ( data2 & 0x00F7);	

		VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

		VK32XX_SPI_Send_Data( data1 );

		VK32XX_CS_HIGH();
	}
}
/***********************************************************************************/




/**********************************
 * ��������VK32XX_UART_SendData;
 * ����  �� Transmits single data through the UARTx peripheral.
 * ����  :uint16_t UARTx,  uint16_t data
 * ���  ����
 * ����  ��VK32XX_UART_SendData( VKCOM1, 0xffff )
 * ע��  ��
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
	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����
	
	VK32XX_SPI_Send_Data( data1 );
	VK32XX_delay(10);
	VK32XX_CS_HIGH();
}
/***********************************************************************************/





/**********************************
 * ��������VK32XX_UART_ReceiveData;
 * ����  �� Returns the most recent received data by the UARTx peripheral.
 * ����  :uint16_t UARTx
 * ���  ����UARTx��FIFO�Ĵ�������ֵ
 * ����  ��VK32XX_UART_ReceiveData( VKCOM1 )
 * ע��  ��
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH(); 

	printf("recv data=%x\r\n",data);
  	return ( data );
}
/***********************************************************************************/




/**********************************
 * ��������VK32XX_Get_IT_UARTx;
 * ����  :��ȡ�Ӵ����жϺ�
 * ����  :��
 * ���  �������Ӵ��ں�
 * ����  ��
 * ע��  ��
*************************************/

uint16_t VK32XX_Get_IT_UARTx( void )
{
	uint16_t addr, data1;
	uint16_t data;

	addr = GIR << 9;
	data1 = addr;

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

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
 * ��������VK32XX_UART_GetFlagStatus;
 * ����  :��ѯ�Ӵ��ڵĸ���״̬
 * ����  :uint16_t UARTx, uint8_t UART_FLAG
 * ���  ��
 * ����  ��VK32XX_UART_GetFlagStatus( VKCOM1, UART_FLAG_OE);
 * ע��  ��
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

  	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����
	printf("read SSR:%x\r\n",data1);
	data = VK32XX_SPI_Receive_Data( data1 );
	printf("read:%x\r\n",data);
	VK32XX_CS_HIGH(); 

	if( (data & UART_FLAG) == UART_FLAG ) 	return SET;
	else 									return RESET;
}
/***********************************************************************************/




/**********************************
 * ��������VK32XX_UART_GetITStatus;
 * ����  :��ѯ�Ӵ��ڵ��ж�״̬
 * ����  :uint16_t UARTx, uint16_t UART_IT
 * ���  ��
 * ����  ��VK32XX_UART_GetITStatus( VKCOM1, UART_IT_FOEIEN );
 * ע��  ��
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
	
	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();

	if( (data & UART_IT)  == UART_IT ) 	return 	SET;
	else								return  RESET;
}
/***********************************************************************************/




	
/**********************************
 * ��������VK32XX_UART_ClearITPendingBit;
 * ����  :����Ӵ��ڵ��жϱ�־λ
 * ����  :uint16_t UARTx, uint16_t UART_IT
 * ���  ��
 * ����  ��VK32XX_UART_ClearITPendingBit( VKCOM1, UART_IT_FOEIEN );
 * ע��  ��
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

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
 * �������� VK32XX_UART_Get_Num_TXFIFO;
 * ����  : ��ѯ����FIFO���ֽ���
 * ����  : uint16_t UARTx
 * ���  ��uint8_t num;
 * ����  ��VK32XX_UART_Get_Num_TXFIFO( VKCOM1 )
 * ע��  ��
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();

	data = data >> 4;

	num = ( (uint8_t) data );
	return num;
}
/***********************************************************************************/





/**********************************
 * �������� VK32XX_UART_Get_Num_RXFIFO;
 * ����  : ��ѯ����FIFO���ֽ���
 * ����  : uint16_t UARTx
 * ���  ��uint8_t num;
 * ����  ��VK32XX_UART_Get_Num_RXFIFO( VKCOM1 )
 * ע��  ��
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	VK32XX_CS_HIGH();

	data = data & 0X000F;

	num = ( (uint8_t) data );
	return num;
}
/***********************************************************************************/




/**********************************
 * �������� VK32XX_UART_Clear_TXFIFO
 * ����  : ��շ���FIFO
 * ����  : uint16_t UARTx
 * ���  ����
 * ����  ��VK32XX_UART_Clear_TXFIFO( VKCOM1 )
 * ע��  ��
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����
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

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	data = data & 0x00fd;

	data1 = 0x8000 | ch | addr | data;

	VK32XX_SPI_Send_Data( data1 );
	VK32XX_delay(10);
	VK32XX_CS_HIGH();
}
/***********************************************************************************/


	


/**********************************
 * �������� VK32XX_UART_Clear_RXFIFO
 * ����  : ��ս���FIFO
 * ����  : uint16_t UARTx
 * ���  ����
 * ����  ��VK32XX_UART_Clear_RXFIFO( VKCOM1 )
 * ע��  ��
*************************************/

void VK32XX_UART_Clear_RXFIFO( uint16_t UARTx )
{
	uint16_t ch, addr, data1, data;

	/* Check the parameters */
  	assert_param(IS_UART_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFOCR << 9;
	data1 = ch | addr;

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	data = data | 0x0001;

	data1 = 0x8000 | ch | addr | data;
	printf("SFOCR:%x\r\n",data1);
	VK32XX_SPI_Send_Data( data1 );

	VK32XX_CS_HIGH();

	while( VK32XX_UART_GetFlagStatus(  UARTx, UART_FLAG_RFEM ) == RESET );

	data1 = ch | addr;

	VK32XX_CS_LOW();	//ʹ��VK3224��SPI����

	data = VK32XX_SPI_Receive_Data( data1 );

	data = data & 0x00fE;

	data1 = 0x8000 | ch | addr | data;

	VK32XX_SPI_Send_Data( data1 );

	VK32XX_CS_HIGH();
}
/***********************************************************************************/









