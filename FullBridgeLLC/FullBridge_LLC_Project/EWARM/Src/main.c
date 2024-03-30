/**
  ******************************************************************************
  * @file    main.c 
  * @author  System Lab
  * @version V2.0.0
  * @date    23-May-2017
  * @brief   Main file for DSMPS project
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "stm32f3xx_hal.h"         
#include "LLC_control_param.h"
#include "LLC_board_config_param.h"
#include "LLC_Globals.h"
#include "LLC_Init_Periph.h"
#include "StateMachine.h"
#include "Control_Layer.h"
#include "Fault_Processing.h"

#ifdef DSMPS_COMMUNICATION
  #include "DPS_Communication.h"
#endif

#ifdef UI_COMMUNICATION
  #include "UI_UART_Interface.h"
#endif

/** @addtogroup DSMPS_project
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/**
  * @brief  Main program 
  * @param  None
  * @retval None
  */
int main(void)
{
  
  /* STM32F3xx HAL library initialization:
       - Configure the Flash prefetch
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to have a system clock = 72 MHz */
  SystemClock_Config();
       
  /* Initialize GPIO pins for digital outputs */
  GPIO_Config();
  
#ifdef OVERCURRENT_PROTECTION 
  /* Configures DAC_OC to as non inverting input of COMP_OC */
  DAC_OverCurrProtection_Config();
  /* Initialize COMP_OC for Overcurrent protection */
  COMP_OverCurrProtection_Config();
#endif
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION
  /* Configures DAC_SR to as non inverting input of COMP_SR1 */
  DAC_AdaptiveSR_Config();
  /* Initialize COMP_SR for adaptive SR */
  COMP_AdaptiveSR_Config();
#endif
  
  /* Initialize ADC to be triggered by the HRTIMER */
  ADC_Config(); 
     
  /* Initialize HRTIM and related I/Os afterwards */
  HRTIM_Config();
  
  /* Initialize TIM6 to schedule Vout control loop */
  TIM6_VoutControl_Config();
  
  /* Initialize TIM16 to schedule low frequency task: temperature and Vin filtering, fan speed control, update config parameters and manage enable/disable of SR and Burst mode */
  TIM16_LowFrequencyTask_Config();

#ifdef FAN_PWM_DRIVING
  FAN_TIM_Config();
  CTR_FanEnable();
#endif
  
#ifdef DSMPS_COMMUNICATION
  /* Initialize USART communication between two boards */
  USART_Config();
#endif
  
#ifdef UI_COMMUNICATION
  /* Initialize UART for User Interface */
  UI_UART_Config();
  /* send begin message */  
  UI_UART_SendBeginMsg();  
  /* prepare reception with DMA */
  if(HAL_UART_Receive_DMA(&UI_UartHandle, &cUI_UartRxLastChar, 1) != HAL_OK)
  {
    while(1); // error handler
  }
#endif
  
  /* init state machine, LEDs, PIDs, etc. */
  CTR_InitEnvironment();
        
  /* Infinite loop */
  while (1)
  {
    /* ---------------- */
    /* Fault management */
    /* ---------------- */
    
    /* Check if a fault has occurred */
    FLT_FaultCheck();
      
    /* Execute State Machine */
    STM_StateMachineTask();
        
    /* manage LED Blinking or ON/OFF */
    LED_Task(&FaultLED);
    LED_Task(&StatusLED);
    
#ifdef DSMPS_COMMUNICATION
    /* manage USART communication - debug for secondary MCU */
//    USART_Task('X');
#endif
  }
}

/** @addtogroup Callbacks
  * @{
  */

/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt 
  *         is generated
  * @retval None
  */
static void TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
}

/**
* @brief  HAL_HRTIM_RepetitionEventCallback
* @param  None
* @retval None
*/
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx)
{    
}

/**
  * @}
  */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
