/**
  ******************************************************************************
  * @file    DPS_Communication.c
  * @author  IMS Systems Lab 
  * @version V2.0.0
  * @date    07-Jul-2014
  * @brief   DC/DC Communication layer.
  *          This file provides fuctions to manage communication beetween primary and secondary
  *				with STM32Cube HAL libs
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
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Select Primary or Secondary communication selected in "DSP_Communication.h" file */


/* Includes ------------------------------------------------------------------*/
#include "DPS_Communication.h"
#include "stm32f3xx_hal.h"
#include "LLC_Globals.h"     //uncomment if this module is used for DC/DC
    
//#include "PFC_Globals.h"      //uncomment if this module is used for PFC
//#include "DCDC_Globals.h"     //uncomment if this module is used for DC/DC

//#include "PFC_pwm_adc_prm.h" // for GPIO toggle debug only
//#include "DCDC_pwm_adc_prm.h" // for GPIO toggle debug only

/** @addtogroup DSMPS_project
  * @{
  */

/** @addtogroup Communication
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_COMMUNICATION_TIME_INTERVAL_MS       ((uint16_t)500)  /**< duration in ms of interval beetween two USART communications */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t UsartTxBuffer[USARTx_TX_BUFFER_SIZE];   /**< Transmit message buffer */
uint8_t UsartRxBuffer[USARTx_RX_BUFFER_SIZE];   /**< Receive message buffer */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
static TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }
  
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
  * @brief  Receives Status from primary MCU (for secondary MCU)
  * @param  None
  * @retval Primary status flag
  */
uint16_t ReceivePrimaryStatus(void)
{
  uint16_t hReceivedStatus = PFC_COMMUNICATION_ERROR;
    
  if(UsartRxBuffer[0] ==  PRIMARY_HEADER){
     hReceivedStatus = (uint16_t)UsartRxBuffer[1];
   }

   /* In this case was arrived the data before the header */
   if(UsartRxBuffer[1] ==  PRIMARY_HEADER){ 
     /* reenable DMA receive message */
     HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)(&UsartRxBuffer[1]), 1);  
     hReceivedStatus = PFC_COMMUNICATION_ERROR;
   }
   else{
     /* reenable DMA receive message */
     HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)UsartRxBuffer, USARTx_RX_BUFFER_SIZE); 
   }

   return hReceivedStatus;
}


/**
  * @brief  Sends Status to secondary MCU (for primary MCU)
  * @param  Primary status flag
  * @retval None
  */
void SendPrimaryStatus(uint8_t bPrimaryStatus2Send)
{
  /* initialize message */
//  UsartTxBuffer[0] = PRIMARY_HEADER;    // if header doesn't change, this instruction is useless
  UsartTxBuffer[1] = bPrimaryStatus2Send;
  
  /* enable DMA transmit message */
  HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)UsartTxBuffer, USARTx_TX_BUFFER_SIZE); 
}

/**
  * @brief  Sends Ack to primary MCU (for secondary MCU) - NOT USED
  * @param  Primary status flag
  * @retval None
  */
void SendAck(uint8_t AckResponse)
{ 
  /* IMPLEMENT HERE THE ACK SENDING FOR STM32F334 WITH HAL_DRIVERS IF NEEDED (see SendPrimaryStatus function)*/
}

/**
  * @brief  Sends a debug message with a counter
  * @param  None
  * @retval None
  */
void SendDebugMessage(void)
{
  static uint8_t bByteToSend = 0; // debug msg
  
  /* initialize message */
//  UsartTxBuffer[0] = 'A';    // if header doesn't change, this instruction is useless
  UsartTxBuffer[0] = 'B';    // if header doesn't change, this instruction is useless
  UsartTxBuffer[1] = bByteToSend;
  
  bByteToSend++;
  
  /* enable DMA transmit message */
  HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)UsartTxBuffer, USARTx_TX_BUFFER_SIZE); 
}

/**
  * @brief  USART task
  * @param  Primary status flag
  * @retval None
  * 
  * Check if is passed the specified time interval from last transmission and send the primary status
  *
  */
void USART_Task(uint8_t bPrimaryStatus2Send){
  if(hCommunicationTimeLeft == 0){
//    HAL_GPIO_WritePin(DEBUG_DIGITAL_OUT_GPIO_PORT, DEBUG_DIGITAL_OUT_GPIO_PIN, GPIO_PIN_SET);
//    SendPrimaryStatus(bPrimaryStatus2Send);
    SendDebugMessage();
    hCommunicationTimeLeft = USART_COMMUNICATION_TIME_INTERVAL_MS;
//    HAL_GPIO_WritePin(DEBUG_DIGITAL_OUT_GPIO_PORT, DEBUG_DIGITAL_OUT_GPIO_PIN, GPIO_PIN_RESET);
  }  
}

/**
  * @}
  */ 

/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************/
