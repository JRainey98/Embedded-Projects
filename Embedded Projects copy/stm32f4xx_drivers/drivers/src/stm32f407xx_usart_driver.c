
/*
 * Mastering MCU#1 Udemy
 *
 */
#include "stm32f407xx_usart_driver.h"

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Sets up the baud rate for USART communication protocol.
 *
 * @param[in]         - USART_RegDef_t pointer
 * @param[in]         - BaudRate to set

 * @return            - None
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	uint32_t PCLKx;

	uint32_t usartdiv;

	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  M_part = usartdiv/100;

  tempreg |= M_part << 4;

  F_part = (usartdiv - (M_part * 100));

  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  tempreg |= F_part;

  pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes the USART peripheral
 *
 * @param[in]         - USART_Handle_t pointer
 *
 * @return            - none
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	uint32_t tempreg=0;

	 USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);

	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= ( 1 << USART_CR1_PCE);

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
	    tempreg |= ( 1 << USART_CR1_PCE);

	    tempreg |= ( 1 << USART_CR1_PS);

	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;
	tempreg=0;

	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->CR2 = tempreg;

	tempreg=0;

	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

}

/*********************************************************************
 * @fn      		  - USART_EnableOrDisable
 *
 * @brief             - Enables or disables the USART peripheral by modifying CR1 
 *
 * @param[in]         - USART_RegDef_t pointer
 * @param[in]         - ENABLE or DISABLE macro

 * @return            - none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}

}



/*********************************************************************
 * @fn      		  - USART_PeriClockControl()
 *
 * @brief             - Uses function macros to enable or disable the USART# peripheral
 *
 * @param[in]         - USART_RegDef_t pointer
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            - none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_DIS();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_DIS();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_DIS();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_DIS();
		}
	}

}
/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Checks the SR register to return the bit value stored in the StatusFlagName specified
 *
 * @param[in]         - USART_RegDef_t pointer
 * @param[in]         - Flag name bit position macro for status register
 *
 * @return            - none
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

   return RESET;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Sends data in polling based mode using UART protocol
 *
 * @param[in]         - USART_Handle_t handle
 * @param[in]         - Pointer to the buffer holding data to be transferred
 * @param[in]         - Lenth, in bytes, of data to be transferred
 *
 * @return            - none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;

	for(uint32_t i = 0 ; i < Len; i++)
	{
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Recieves data using USART protocol. Polling based
 *
 * @param[in]         - USART_Handle_t pointer
 * @param[in]         - pointer to buffer where recieved byte will be stored for reading
 * @param[in]         - Length of data, in bytes, to be received.
 *
 * @return            - none
 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{

				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{

				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			pRxBuffer++;
		}
	}

}

/*
 * @fn      		  - USART_SendDataIT()
 *
 * @brief             - Sends Len amount of bytes via USART protocol using interrupts.
 *
 * @param[in]         - USART_Handle_t pointer
 * @param[in]         - pointer to buffer that holds data to transmit
 * @param[in]         - Number of bytes that will be transfereed.
 *
 * @return            - txstate macro
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);

		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}
	return txstate;
}


/*
 * @fn      		  - USART_ReceiveDataIT()
 *
 * @brief             - Recieves Len amount of bytes using USART communication protocol in interrupt-mode.
 *
 * @param[in]         - USART_Handle_t pointer
 * @param[in]         - pointer to buffer that will hold data received
 * @param[in]         - Len, in bytes, of data that will be received.
 *
 * @return            - rxstate macro
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR; // dummyread clears the data register

		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}


/*
 * @fn      		  - USART_ClearFlag()
 *
 * @brief             - Clears certain flags set in the SR register for USART
 *
 * @param[in]         - USART_RegDef_t pointer
 * @param[in]         - USART flag name macro
 *
 * @return            - none
 *
 * @Note              - Applicable to only USART_CTS_FLAG , USART_LBD_FLAG
 * USART_TC_FLAG,USART_TC_FLAG flags
 */

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);

}

/*
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - Enables or disables USART interrupts.

 * @param[in]         - IRQNumber to enable or disable
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) 
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - Assigns a priority number to interrupts based on IRQ number
 *
 * @param[in]         - USART IRQ number
 * @param[in]         - Priority to assign to IRQ number
 *
 * @return            - none
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/*
 * @fn      		  - USART_IRQHandler
 *
 * @brief             - Function determines what kind of USART interrupt was triggered
 *							
 * @param[in]         - USART_Handle_t pointer

 * @return            - None
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(! pUSARTHandle->TxLen )
			{
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
				pUSARTHandle->TxBusyState = USART_READY;
				pUSARTHandle->pTxBuffer = NULL;
				pUSARTHandle->TxLen = 0;

				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}


	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(pUSARTHandle->TxLen > 0)
			{
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{

					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{

						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{

					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{

						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}

					else
					{
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}


			}

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//CTS is not applicable for UART4 and UART5 peripherals

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Checking for IDLE flag*******************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}



/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - To be implemented by the user
 *
 * @param[in]         - USART_Handle_t pointer
 *
 * @return            - none
 *
 * @Note              - Needs to be replaced, weak function
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}
