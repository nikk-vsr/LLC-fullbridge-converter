/**
  ******************************************************************************
  * @file    UI_UART_Interface.c
  * @author  IMS Systems Lab and Technical Marketing - Motion Control group
  * @version V1.0.0
  * @date    26-November-2015
  * @brief   This module manages UART communication with a PC to execute commands or
  *			 set/get control parameters
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/   
#include "stm32f3xx_hal.h"
#include "UI_UART_Interface.h"
#include "y.tab.h"

/** @addtogroup MAIN_GROUP< OPTIONAL >
  * @{
  */

/** @defgroup SUB_GROUP< OPTIONAL >
  * @brief 
  * @{
  */


/* Private typedef -----------------------------------------------------------*/

#ifndef __cplusplus
typedef enum {FALSE = 0, TRUE = !FALSE} bool; 
#endif

/* Private defines -----------------------------------------------------------*/
//#define UI_UART_BAUD_RATE    115200
#define UI_UART_BAUD_RATE    57600
//#define UI_UART_BAUD_RATE    38400
//#define UI_UART_BAUD_RATE    19200
//#define UI_UART_BAUD_RATE    9600 

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UI_UartHandle;
/* Buffer used for UART interface  reception */
uint8_t aUI_UartRxBuffer[UI_UART_RXBUFFERSIZE];
/* Last Char received */
uint8_t cUI_UartRxLastChar;

/* Private function prototypes -----------------------------------------------*/
static void UI_UART_Error_Handler(void);
static UART_TestStatus_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static UART_TestStatus_t AdvBuffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLengthCmd);
static UART_TestStatus_t AdvBuffercmp2(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLengthCmd, uint16_t* pIndexBuffer);
bool CheckOutOfRange(int32_t hParameter, int32_t hLowerlimit, int32_t hUpperlimit);

/* Extern function prototypes ------------------------------------------------*/
extern void yyinit(void);
extern int yylex(void);

/* Imported variables from lexer ---------------------------------------------*/
extern char retVal[];
extern const char help_msg[];


/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  Configure USART to comunicate with PC for user interface commands
  * @param  None
  * @retval None
  */
void UI_UART_Config(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = UI_UART_BAUD_RATE baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UI_UartHandle.Instance        = UI_USART;
  UI_UartHandle.Init.BaudRate   = UI_UART_BAUD_RATE;
  UI_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UI_UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UI_UartHandle.Init.Parity     = UART_PARITY_NONE;
  UI_UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UI_UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UI_UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if(HAL_UART_Init(&UI_UartHandle) != HAL_OK)
  {
    UI_UART_Error_Handler();
  }
  
//  /* prepare reception with DMA */
//  if(HAL_UART_Receive_DMA(&UI_UartHandle, &cUI_UartRxLastChar, 1) != HAL_OK)
//  {
//    UI_UART_Error_Handler();
//  }
 }

/**
  * @brief UART PC Interface MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void UI_UART_MspInit(UART_HandleTypeDef *huart)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  UI_UART_TX_GPIO_CLK_ENABLE();
  UI_UART_RX_GPIO_CLK_ENABLE();

  /* Enable USARTx clock */
  UI_UART_CLK_ENABLE(); 

  /* Enable DMA clock */
  UI_UART_DMA_CLK_ENABLE();   
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = UI_UART_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = UI_UART_TX_AF;
  
  HAL_GPIO_Init(UI_UART_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = UI_UART_RX_PIN;
  GPIO_InitStruct.Alternate = UI_UART_RX_AF;
    
  HAL_GPIO_Init(UI_UART_RX_GPIO_PORT, &GPIO_InitStruct);
    
  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = UI_UART_TX_DMA_STREAM;  
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  
  HAL_DMA_Init(&hdma_tx);   
  
  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(huart, hdmatx, hdma_tx);
    
  /* Configure the DMA handler for reception process */
  hdma_rx.Instance                 = UI_UART_RX_DMA_STREAM;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(huart, hdmarx, hdma_rx);
    
  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (UI_UART_TX) */
  HAL_NVIC_SetPriority(UI_UART_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(UI_UART_DMA_TX_IRQn);
  
  /* Disable the transfer complete interrupt */
//  __HAL_DMA_DISABLE_IT(UI_UartHandle.hdmarx, DMA_IT_TC);
    
  /* NVIC configuration for DMA transfer complete interrupt (UI_UART_RX) */
  HAL_NVIC_SetPriority(UI_UART_DMA_RX_IRQn, 0, 0);   
  HAL_NVIC_EnableIRQ(UI_UART_DMA_RX_IRQn);
  
  /* NVIC for USART, to catch the TX complete */
//  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
//  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
  * @brief UART PC Interface MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void UI_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  UI_UART_FORCE_RESET();
  UI_UART_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(UI_UART_TX_GPIO_PORT, UI_UART_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(UI_UART_RX_GPIO_PORT, UI_UART_RX_PIN);
   
  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA channel associated to reception process */
  if(huart->hdmarx != 0)
  {
    HAL_DMA_DeInit(huart->hdmarx);
  }
  /* De-Initialize the DMA channel associated to transmission process */
  if(huart->hdmatx != 0)
  {
    HAL_DMA_DeInit(huart->hdmatx);
  }  
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(UI_UART_DMA_TX_IRQn);
  HAL_NVIC_DisableIRQ(UI_UART_DMA_RX_IRQn);
}

void UI_UART_SendBeginMsg(void)
{
  sprintf(retVal, help_msg);
  HAL_UART_Transmit_DMA(&UI_UartHandle, (uint8_t*)retVal, strlen(retVal)); 

}

/**
  * @brief  This function check if the sent parameter is within the limits
  * @param  parameter to check, upper and lower limits
  * @retval TRUE if the parameters is outside the range, FALSE otherwise
  */
bool CheckOutOfRange(int32_t hParameter, int32_t hLowerlimit, int32_t hUpperlimit){
  
  if((hParameter > hUpperlimit)||(hParameter < hLowerlimit))
  {
    return TRUE;
  }
  else{
    return FALSE;
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED  : pBuffer1 identical to pBuffer2
  *         FAILED : pBuffer1 differs from pBuffer2
  */
static UART_TestStatus_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
  * @brief  Compares two buffers without spaces for and distinguish upper and lower case (pBuffer2 contains only lower case).
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED  : pBuffer1 identical to pBuffer2
  *         FAILED : pBuffer1 differs from pBuffer2
  */
static UART_TestStatus_t AdvBuffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLengthCmd)
{
  while(BufferLengthCmd--)
  {
    while (*pBuffer1 == ' '){                           // eliminate spaces from pBuffer1
      pBuffer1++;
    }
//    if(tolower(*pBuffer1) != tolower(*pBuffer2))      // convert both pBuffer1 and pBuffer2 in lower case
    if(tolower(*pBuffer1) != *pBuffer2)  
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
  * @brief  Compares two buffers without spaces and distinguish upper and lower case (pBuffer2 contains only lower case).
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED  : pBuffer1 identical to pBuffer2 and update pIndexBuffer value
  *         FAILED : pBuffer1 differs from pBuffer2
  */
static UART_TestStatus_t AdvBuffercmp2(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLengthCmd, uint16_t* pIndexBuffer)
{
  uint16_t nBuffer1Length = *pIndexBuffer;// returns the index of the array to compare next string
  while(BufferLengthCmd--)
  {
    while (*pBuffer1 == ' '){                           // eliminate spaces from pBuffer1
      pBuffer1++;
      nBuffer1Length++;
    }
//    if(tolower(*pBuffer1) != tolower(*pBuffer2))      // convert both pBuffer1 and pBuffer2 in lower case
    if(tolower(*pBuffer1) != *pBuffer2)  
    {
      return FAILED;
    }
    nBuffer1Length++;
    pBuffer1++;    
    pBuffer2++;
  }
  
  *pIndexBuffer = nBuffer1Length; //update scan index
  return PASSED;
}

/**
  * @brief  Fills buffer.
  * @param  pBuffer: pointer on the Buffer to fill
  * @param  BufferLength: size of the buffer to fill
  * @retval None
  */
static void Fill_Buffer(uint8_t *pBuffer, uint16_t BufferLength)
{
  uint16_t index = 0;
  
  /* Put in global buffer same values */
  for (index = 0; index < BufferLength; index++ )
  {
    pBuffer[index] = 0x00;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void UI_UART_Error_Handler(void)
{
  while(1)
  {
  }  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  static uint8_t nRxBuffIndex = 0;  
     
  HAL_UART_Receive_DMA(&UI_UartHandle, &cUI_UartRxLastChar, 1);
  
  if(*(UartHandle->pRxBuffPtr) != USART_ENTER_CHAR){
    if(*(UartHandle->pRxBuffPtr) != USART_BACKSPACE_CHAR){
      aUI_UartRxBuffer[nRxBuffIndex] = *(UartHandle->pRxBuffPtr);
      nRxBuffIndex++;
    }
    else{
      /* discard backspace character, decrease the buffer index and print empty space */
      nRxBuffIndex--;
      HAL_UART_Transmit_DMA(&UI_UartHandle, (uint8_t*)aUartDeleteCharMsg, 2);
    }
  }
  else{
    yyinit();
    HAL_UART_Transmit_DMA(&UI_UartHandle, retVal, strlen(retVal));
    Fill_Buffer(&aUI_UartRxBuffer[0], nRxBuffIndex);
    nRxBuffIndex = 0;   // reset Rx buff index
  }
}

/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  */
void UI_UART_DMA_RX_IRQHandler(void)
{
//  HAL_DMA_IRQHandler(UI_UartHandle.hdmarx);
  
  /* ------- instructions instead of HAL_DMA_IRQHandler() calling ------- */
  /* Disable the half, full and error transfer interrupt */
  __HAL_DMA_DISABLE_IT(UI_UartHandle.hdmarx, DMA_IT_HT|DMA_IT_TE|DMA_IT_TC);  
  /* Clear the half transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(UI_UartHandle.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(UI_UartHandle.hdmarx));
  /* Clear the transfer complete flag */
   __HAL_DMA_CLEAR_FLAG(UI_UartHandle.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(UI_UartHandle.hdmarx));

/* Update error code */
//      SET_BIT(hdma->ErrorCode, HAL_DMA_ERROR_NONE);

  /* Change the DMA state */
  UI_UartHandle.hdmarx->State = HAL_DMA_STATE_READY;  
  /* Process Unlocked */
  __HAL_UNLOCK(UI_UartHandle.hdmarx);      
  UI_UartHandle.RxXferCount = 0;

  /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
  in the UART CR3 register */
  UI_UartHandle.Instance->CR3 &= (uint32_t)~((uint32_t)USART_CR3_DMAR);

  /* At end of Rx process, restore UI_UartHandle->RxState to Ready */
  UI_UartHandle.RxState = HAL_UART_STATE_READY;

  /* Call UART Rx complete callback */
  HAL_UART_RxCpltCallback(&UI_UartHandle);
  /* ------------------------------------------------------------------------ */

}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA stream 
  *         used for USART data reception    
  */
void UI_UART_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UI_UartHandle.hdmatx);
  
  /* Tx process is ended, restore huart->gState to Ready */
  UI_UartHandle.gState = HAL_UART_STATE_READY;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
