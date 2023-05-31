/**
  ******************************************************************************
  * @file    StateMachine.c 
  * @author  System Lab
  * @version V1.0.0
  * @date    26-May-2016
  * @brief   This file provides DSMPS state machine task
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
#include "stm32f3xx_hal.h"
#include "DSMPS_type.h"
#include "LLC_control_param.h"
#include "LLC_board_config_param.h"
#include "LLC_Globals.h"
#include "Fault_processing.h"
#include "Control_Layer.h"  
#include "StateMachine.h"

/** @addtogroup DSMPS_project
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define VOUT_MIN_CLOSED_LOOP    OUT_VOLT_ADC_VALUE(46)//47  /**< minimum voltage to close the control loop during the start-up phase */
     
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile DSMPS_State_t DSMPS_State;              /**< DSMPS state variable */
static volatile bool bStartUpComplete = FALSE;          /**< Start-up complete flag */

#ifdef START_WITHOUT_COMMAND
  static volatile bool bConverterEnabled = TRUE;        /**< Enable conversion flag */
#else
  static volatile bool bConverterEnabled = FALSE;       /**< Enable conversion flag */
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/**
  * @brief  Executes converter's state machine task
  * @param  None
  * @retval None
  */
void STM_StateMachineTask(void)
{    
    /**### Global State Machine */
    switch(DSMPS_State){   
                     
    case DSMPS_IDLE:    /** @arg  \a DSMPS_IDLE state:  wait for input voltage conditions */            
      if(STM_GetConverterEnabledFag())
      {
        STM_SetStateMachineStatus(DSMPS_INIT);      // comment to block state machine in IDLE state until a debugger command
      }
      break;
    
    case DSMPS_INIT:    /** @arg  \a DSMPS_INIT state: reinitialize control variables and enable output */
      /* reinit control variables */
      CTR_InitControlParameters();

#ifdef OVERCURRENT_PROTECTION
        /* Disable Overcurrent protection fault at next start-up only if STOP state is set by communication or debugger */
        HAL_HRTIM_FaultModeCtl(&hhrtim, HRTIM_FAULT_OC, HRTIM_FAULTMODECTL_DISABLED);
#endif

      /* Enable Full Bridge Gate drivers */
//      CTR_GateDriverEnable(); // gate driver enable pins are pull-upped in the board        
      /* Enable PWMs */
      CTR_PWMOutputEnable(&hhrtim);        // HRTIM - High resolution Timer
      
      /* set delay timer for Vout ramp-up */
      SetDelayTime(&hDSMPS_StartUp_TimeLeft, DSMPS_STARTUP_TIME_DURATION_mS);
      /* set fast LED blinking until rump up is complete */
      LED_SetParams(&StatusLED, LED_BLINK_INF, 0, LED_BLINK_PERIOD_SHORT_MS, LED_BLINK_REPETITION_PERIOD_MS);      
      
      /* set next state */
      STM_SetStateMachineStatus(DSMPS_START);
      break;      
            
    case DSMPS_START:     /**  @arg \a DSMPS_START state: ramp-up Vout for DC/DC regulation */
           
      /* only if open loop mode is disabled, check the output voltage during ramp-up and enable start-up failed procedure, if defined */
      if(DCDC_ConfigParamStruct.bOpenLoopEnabled == FALSE)
      {
        
#ifdef START_UP_FAILED_ENABLED
        /* if start-up time is elapsed, enable undervoltage check so if bus can be charged the system goes in DSMPS_STOP state */
        if(DelayTimeIsElapsed(&hDSMPS_StartUp_TimeLeft) == TRUE){        
          /* call start-up failed procedure */        
          FLT_StartUpFailedProcedure();
        }
#endif
        /* Check if Vout is greater than Vout_min to close feedback loop and go to DSMPS_RUN state */
        if(hVoutVoltageFiltered > VOUT_MIN_CLOSED_LOOP)
        {
          /* update the controller output to avoid discontinuity of frequency at closing of control loop: 
         integral term is set equal to last PWM period multiplied by ki divisor, then if error_k = 0 the f_closed_loop_k =  f_open_loop_k-1 */
         CTR_UpdateRegulatorOutput();                  
         /* reference voltage at final start-up value */
         hVout_Reference = STARTUP_FINAL_OUT_VOLTAGE;
         /* set ramp-up complete flag */
         bStartUpComplete = TRUE;
         /* enable undervoltage check */
         bCheckUndervoltageEnabled = TRUE;
         /* enable out overcurrent check */
         bCheckOutOvercurrentEnabled = TRUE;
         /* enable overtemperature check */
         bCheckOvertemperatureEnabled = TRUE;   
         /* set status LED ON */
         LED_SetParams(&StatusLED, LED_ON, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
         
  #ifdef OVERCURRENT_PROTECTION
          /* Enable Overcurrent protection fault */
          HAL_HRTIM_FaultModeCtl(&hhrtim, HRTIM_FAULT_OC, HRTIM_FAULTMODECTL_ENABLED);
  #endif        
          /* set next state */
          STM_SetStateMachineStatus(DSMPS_RUN);
          }
      }
      else{ //open loop mode enable, go directly in RUN state after start-up time
        if(DelayTimeIsElapsed(&hDSMPS_StartUp_TimeLeft) == TRUE)
        {
          /* set ramp-up complete flag */
          bStartUpComplete = TRUE;
          /* undervoltage check disabled in open loop mode */
          bCheckUndervoltageEnabled = FALSE;
          /* enable overtemperature check */
          bCheckOvertemperatureEnabled = TRUE;   
          /* set status LED ON */
          LED_SetParams(&StatusLED, LED_ON, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
         
  #ifdef OVERCURRENT_PROTECTION
          /* Enable Overcurrent protection fault */
          HAL_HRTIM_FaultModeCtl(&hhrtim, HRTIM_FAULT_OC, HRTIM_FAULTMODECTL_ENABLED);  // OC fault enable and fault mode enable
  #endif        
          /* set next state */
          STM_SetStateMachineStatus(DSMPS_RUN);
        }
      }
      break;
        
    case DSMPS_RUN:           /** @arg \a RUN state: normal operation of the converter */  
      break;  
        
    case DSMPS_STOP:          /** @arg \a DSMPS_STOP state: stop of conversion if any fault has occurred */
        /* Disable all PWMs outputs */
        CTR_PWMAllOutputsDisable(&hhrtim);
              
#ifdef SYNCH_RECTIFICATION
        /* Disable gate drivers for SR PWMs */
        CTR_SRGateDriverDisable();
#endif

#ifdef LIGHT_LOAD_BURST_MODE
      /* call burst mode control fnc with a current greather then the threshold needed to set BMRange at BM_NOBURST value */
//      CTR_BurstModeControl(BURST_MODE_IOUT_RANGE3_TH_H + 1, &hhrtim); // 3 THRESHOLDS
//        CTR_BurstModeControl(BURST_MODE_IOUT_RANGE23_TH_H + 1, &hhrtim); // 2 THRESHOLDS
        CTR_BurstModeControl(BURST_MODE_IOUT_RANGE12_TH_H + 1, &hhrtim); // 1 THRESHOLD
      /* disable burst mode */
      CTR_BurstModeDisable(&hhrtim);
#endif
      
      /* disable undervoltage check */
      bCheckUndervoltageEnabled = FALSE;
      /* disable undervoltage check */
      bCheckOutOvercurrentEnabled = FALSE;
      /* disable overtemperature check */
      bCheckOvertemperatureEnabled = FALSE;
      /* reset start-up complete flag */
      bStartUpComplete = FALSE;
#ifdef OUT_VOLTAGE_BURST_MODE      
      /* reset burst mode variable to avoid PWM restart */
      bVoutBurstModeEnabled = FALSE;
#endif      
      /* check if there was an error, then update the last fault variable and set next state */
      if(FLT_GetSystemFault() != DCDC_NO_ERROR){ // Check for errors
        /* store the last fault variable */
        FLT_StoreLastSystemFault();
        /* Reset enable flag variable - to disable for autorestart */
        STM_SetConverterEnabledFag(FALSE);
        /* set status LED OFF */
        LED_SetParams(&StatusLED, LED_OFF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
        /* set next state - to manage if STOP state is set by communication */
        STM_SetStateMachineStatus(DSMPS_FAULT);
      }
      else{
        /* Reset enable flag variable */
        STM_SetConverterEnabledFag(FALSE);
        /* set status LED OFF */
        LED_SetParams(&StatusLED, LED_OFF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
        /* set next state */
        STM_SetStateMachineStatus(DSMPS_WAIT);
      }      
      break;
      
    case DSMPS_FAULT:         /** @arg \a DSMPS_FAULT state: persistent state after a fault */
      // TO INSERT HERE PROCEDURE TO CLEAR FAULTS LIKE OVERCURRENT AND START-UP FAILED IF NEEDED: they are not recoverable
      /* Disable all PWMs outputs */
      CTR_PWMAllOutputsDisable(&hhrtim);
        
      if (FLT_GetSystemFault() == DCDC_NO_ERROR){
        /* Overcurrent Fault is not recoverable */
        /* set delay timer for wait status */
        SetDelayTime(&hDSMPS_Wait_TimeLeft, DSMPS_WAIT_TIME_DURATION_mS);   
        /* set next state */
        STM_SetStateMachineStatus(DSMPS_WAIT);
      }      
      break;
      
    case DSMPS_WAIT:    /** @arg  \a DSMPS_WAIT state:  wait a delay time before return in DSMPS_IDLE state */
      if (DelayTimeIsElapsed(&hDSMPS_Wait_TimeLeft) == TRUE){
        /* Set Status LED blinking */
        LED_SetParams(&StatusLED, LED_BLINK_INF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
        /* set fault LED off */
        LED_SetParams(&FaultLED, LED_OFF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
        /* check configuration enable flag that can be set by UI to reenable the converter */
        if(DCDC_ConfigParamStruct.bConverterEnabled == TRUE){
          /* set convert enable flag */
          STM_SetConverterEnabledFag(TRUE);
        }
        /* set next state */
        STM_SetStateMachineStatus(DSMPS_IDLE);
      }
      break;
      
  }
  /*-- end of switch-case State Machine --*/ 

}

/**
  * @brief  Set State machine status
  * @param  DSMPS_NewState: new state 
  * @retval None
  */
void STM_SetStateMachineStatus(DSMPS_State_t DSMPS_NewState)
{
  DSMPS_State = DSMPS_NewState;
}

/**
  * @brief  Get State machine status
  * @param  None
  * @retval State machine status
  */
DSMPS_State_t STM_GetStateMachineStatus(void)
{
  return DSMPS_State;
}

/**
  * @brief  Set converter enable flag
  * @param  bState: new enable flag state
  * @retval None
  */
void STM_SetConverterEnabledFag(bool bState)
{
  bConverterEnabled = bState;
}

/**
  * @brief  Get converter enable flag
  * @param  None
  * @retval bool: enable flag state
  */
bool STM_GetConverterEnabledFag(void)
{
  return bConverterEnabled;
}

/** @addtogroup LED_Management
  * @{
  */

/**
  * @brief  Turns LED On.
  * @param  LED: Pointer to LED object
  * @retval None
  */
void LED_On(LED_Struct_t* LED)
{  
  HAL_GPIO_WritePin(LED->LED_GPIOPort, LED->LED_GPIOPin, GPIO_PIN_SET);
  LED->bActualOn = TRUE;  
}

/**
  * @brief  Turns LED Off.
  * @param  LED: Pointer to LED object
  * @retval None
  */
void LED_Off(LED_Struct_t* LED)
{
  HAL_GPIO_WritePin(LED->LED_GPIOPort, LED->LED_GPIOPin, GPIO_PIN_RESET);
  LED->bActualOn = FALSE;
}

/**
  * @brief  Toggle LED.
  * @param  LED: Pointer to LED object
  * @retval None
  */
void LED_Toggle(LED_Struct_t* LED)
{
  HAL_GPIO_TogglePin(LED->LED_GPIOPort, LED->LED_GPIOPin);
  LED->bActualOn = (bool)(1-LED->bActualOn);
}

/**
  * @brief  Decreases LED's blinking counter.
  * @param  LED: Pointer to LED object
  * @retval None
  */
void LED_DecreaseBlinkCounter(LED_Struct_t* LED)
{
  if((LED->hBlinkPeriodCounterLeft != 0)) LED->hBlinkPeriodCounterLeft--;
}

/**
  * @brief  Initialize Status and Fault LEDs
  * @param  LED: Pointer to LED object
  * @param  LED_GPIOPort: GPIO port of LED
  * @param  LED_GPIOPin: GPIO pin of LED
  * @retval None
  */
void LED_Init(LED_Struct_t* LED, GPIO_TypeDef* LED_GPIOPort, uint16_t LED_GPIOPin)
{  
  LED->LED_GPIOPin = LED_GPIOPin;
  LED->LED_GPIOPort = LED_GPIOPort;
  
  /* set LED off and default parameteres */
  LED_SetParams(LED, LED_OFF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
}

/**
  * @brief  Sets params of LED status.
  * @param  LED Pointer to LED object
  * @param  newLEDModality new LED modality, a value of LED_Modality_t
  * @param  nBlinkCounter number of blinking in LED_BLINK_N modality
  * @param  hBlinkPeriodms blinking period in LED_BLINK_N modality
  * @param  hBlinkPeriodms blinking repetition period in LED_BLINK_N modality (stop period before a blink again nBlinkCounter times
  * @retval None
  */
void LED_SetParams(LED_Struct_t* LED, LED_Modality_t newLEDModality, uint8_t nBlinkCounter, uint16_t hBlinkPeriodms, uint16_t hhBlinkRepetitionPeriodms)
{
  LED->LED_Modality = newLEDModality;
  LED->nBlinkCount = nBlinkCounter;
  LED->nBlinkLeftCount = nBlinkCounter;
  LED->hBlinkPeriodms = hBlinkPeriodms;
  LED->hBlinkPeriodCounterLeft = hBlinkPeriodms;
  LED->hBlinkRepetitionPeriodms = hhBlinkRepetitionPeriodms;
}

/**
  * @brief  Manages LED's blinking and ON/OFF.
  * @param  LED Pointer to LED object
  * @retval None
  */
void LED_Task(LED_Struct_t* LED)
{
  switch(LED->LED_Modality)
  {
  case LED_ON:
    if(LED->bActualOn == FALSE){
      LED_On(LED);
    }
    break;
 
  default:
  case LED_OFF:
    if(LED->bActualOn == TRUE){
      LED_Off(LED);
    }
    break;
    
  case LED_BLINK_N:
    if((LED->hBlinkPeriodCounterLeft == 0) && (LED->nBlinkLeftCount != 0)){
      if(LED->bActualOn == TRUE){
        LED_Off(LED);
        LED->nBlinkLeftCount--;
      }      
      else{
        LED_On(LED);        
      }
      LED->hBlinkPeriodCounterLeft = LED->hBlinkPeriodms;
    }
    else if(LED->nBlinkLeftCount == 0){ // reenable blinking after hBlinkRepetitionPeriodms ms
      LED->nBlinkLeftCount = LED->nBlinkCount;
      LED->hBlinkPeriodCounterLeft = LED->hBlinkRepetitionPeriodms;
    }
    break;
    
  case LED_BLINK_INF:
    if(LED->hBlinkPeriodCounterLeft == 0){
      if(LED->bActualOn == TRUE){
        LED_Off(LED);
      }        
      else{
        LED_On(LED);        
      }
      LED->hBlinkPeriodCounterLeft = LED->hBlinkPeriodms;
    }
    break;
  }
  
}
/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
