/**
  ******************************************************************************
  * @file    Control_Layer.c
  * @author  IMS Systems Lab 
  * @version V2.1.0
  * @date    23-May-2017
  * @brief   Control layer.
  *          This file provides fuctions to control the DC/DC stage
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
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "LLC_type.h"
#include "LLC_control_param.h"
#include "LLC_board_config_param.h"
#include "LLC_PWMnCurrVoltFdbk.h"
#include "Control_Layer.h"
#include "LLC_Globals.h"
#include "StateMachine.h"
#include "PID_regulators.h"


/** @addtogroup DSMPS_project
  * @{
  */
    
/** @addtogroup DCDC_Control_Layer
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FAN_PWM_DELTA_DUTY              ((uint16_t)((FAN_PWM_TIM_PERIOD)/100))          /**< delta duty cycle for fan control */
#define FAN_ALPHA_DUTY_LOAD		(int32_t)(((int32_t)((int16_t)FAN_PWM_DUTY_HIGH_SPEED -(int16_t)FAN_PWM_DUTY_LOW_SPEED)*1024) / (int32_t)(FAN_HIGH_LOAD_THRESHOLD-FAN_LOW_LOAD_THRESHOLD))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool bFanEnabled = FALSE;                                      /**< TRUE when Fan is enabled, FALSE otherwise */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function Initialize State machine, LED, PID regulator and config param struct
  * @param  None
  * @retval None 
  *
  */
void CTR_InitEnvironment(void)
{  
#ifdef USE_EXTENDED_PID
  /* initializzation of Vout extended PID */
  DCDC_PID_Init_ex(&PID_Vout_InitStructure);
#else
  /* initializzation of Vout PID */
  DCDC_PID_Init(&PID_Vout_InitStructure);
#endif  
  
  /* Init LEDs and set off state */
  LED_Init(&StatusLED, STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);
  LED_Init(&FaultLED, FAULT_LED_GPIO_PORT, FAULT_LED_GPIO_PIN);
    
  /* filtered variables initialized to the last measured values */
  hTemperatureFiltered = DCDC_MeasureStruct.hTemperature;  
  hVinVoltageFiltered = DCDC_MeasureStruct.hVin;
  hVoutVoltageFiltered = DCDC_MeasureStruct.hVout;
  
#ifdef OUT_CURRENT_SENSOR_CALIBRATION
 hHallIcOffsetCalib = DCDC_MeasureStruct.hIout;
 hSrIoutTurnOnThreshold = PWM_ConvertCurrentValue(SR_IOUT_TURN_ON_THRESHOLD_A);
 hSrIoutTurnOffThreshold = PWM_ConvertCurrentValue(SR_IOUT_TURN_OFF_THRESHOLD_A);
 hBmIoutRange12ThresholdHigh = PWM_ConvertCurrentValue(BURST_MODE_IOUT_RANGE12_TH_H_A);
 hBmIoutRange12ThresholdLow = PWM_ConvertCurrentValue(BURST_MODE_IOUT_RANGE12_TH_L_A);
 hBmIoutRange23ThresholdHigh = PWM_ConvertCurrentValue(BURST_MODE_IOUT_RANGE23_TH_H_A);
 hBmIoutRange23ThresholdLow = PWM_ConvertCurrentValue(BURST_MODE_IOUT_RANGE23_TH_L_A);  
#endif
  
  /* Init configuration param struct ---------------------------------------- */
#ifdef START_WITHOUT_COMMAND
  DCDC_ConfigParamStruct.bConverterEnabled = TRUE;
#else
  DCDC_ConfigParamStruct.bConverterEnabled = FALSE;
#endif
  
#ifdef OPEN_LOOP_MODE
  DCDC_ConfigParamStruct.bOpenLoopEnabled = TRUE;
#else
  DCDC_ConfigParamStruct.bOpenLoopEnabled = FALSE;
#endif
  
#ifdef SYNCH_RECTIFICATION_INIT_ENABLED
  DCDC_ConfigParamStruct.bSREnabled = TRUE;
#else
  DCDC_ConfigParamStruct.bSREnabled = FALSE;
#endif
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION_INIT_ENABLED
  DCDC_ConfigParamStruct.bAdaptiveSREnabled = TRUE;
#else
  DCDC_ConfigParamStruct.bAdaptiveSREnabled = FALSE;
#endif
  
#ifdef FAN_PWM_DRIVING_INIT_ENABLED
  DCDC_ConfigParamStruct.bFanPWMDrivingEnabled = TRUE;
#else
  DCDC_ConfigParamStruct.bFanPWMDrivingEnabled = FALSE;
#endif
  
#ifdef LIGHT_LOAD_BURST_MODE_INIT_ENABLED
  DCDC_ConfigParamStruct.bBurstModeEnabled = TRUE;
#else
  DCDC_ConfigParamStruct.bBurstModeEnabled = FALSE;
#endif
  
  DCDC_ConfigParamStruct.wOpenLoopFreq_Hz = HRTIM_MAX_PWM_FREQ_HZ;
  DCDC_ConfigParamStruct.hFixedSRDelayRising1_ns = SYNCH_RECT_DELAY_RISING1_INIT_NS;
  DCDC_ConfigParamStruct.hFixedSRDelayFalling1_ns = SYNCH_RECT_DELAY_FALLING1_INIT_NS;
  DCDC_ConfigParamStruct.hFixedSRDelayRising2_ns = SYNCH_RECT_DELAY_RISING2_INIT_NS;
  DCDC_ConfigParamStruct.hFixedSRDelayFalling2_ns = SYNCH_RECT_DELAY_FALLING2_INIT_NS;
  DCDC_ConfigParamStruct.hDeadTimeFullBridge_ns = DEAD_TIME_RISING_NS;
  DCDC_ConfigParamStruct.hRegulatorKpGain = PID_VOUT_KP_DEFAULT;
  DCDC_ConfigParamStruct.hRegulatorKiGain = PID_VOUT_KI_DEFAULT;
  
  DCDC_ConfigParamStruct.bConfigurationChanged = TRUE; //update init configuration
  /* ------------------------------------------------------------------------ */
    
  /* set delay time for wait state duration */
  SetDelayTime(&hDSMPS_Wait_TimeLeft, DSMPS_WAIT_TIME_DURATION_mS);
  
  /* set initial state */ 
  STM_SetStateMachineStatus(DSMPS_WAIT);
  
}
  
/**
  * @brief  This function updates the controller output to avoid discontinuity at closing of control loop
  * @param  None
  * @retval None 
  *
  */
void CTR_UpdateRegulatorOutput(void)
{  
  /* prevent that PWM period is outside closed loop boundaries when the control loop is closed */
  if(hPWMPeriod > HRTIM_MAX_PWM_PERIOD)
  {
    hPWMPeriod = HRTIM_MAX_PWM_PERIOD;
  }
  else if(hPWMPeriod < HRTIM_MIN_PWM_PERIOD)
  {
    hPWMPeriod = HRTIM_MIN_PWM_PERIOD;
  }
          
#ifdef USE_EXTENDED_PID
  /* update PI integral term before closing the control loop (multiplication by ki divisor is made inside the function) */
  PID_Set_IntegralTerm_ex(&PID_Vout_InitStructure, hPWMPeriod);
#else
  /* update PI integral term before closing the control loop (multiplication by ki divisor is made inside the function) */
  PID_Set_IntegralTerm(&PID_Vout_InitStructure, hPWMPeriod);
#endif
}

/**
  * @brief  This function Initialize PI object and control variables
  * @param  None
  * @retval None 
  *
  */
void CTR_InitControlParameters(void)
{
  /* set actual and init ramp-up reference at last Vout measure */
  hVout_Reference = DCDC_MeasureStruct.hVout;                                      
  hVout_Reference_init = DCDC_MeasureStruct.hVout; 

#ifdef SYNCH_RECTIFICATION
  /* restore SR init values */
  hSR_DelayRising1 = SYNCH_RECT_DELAY_RISING1_INIT_HR_TICKS;           
  hSR_DelayFalling1 = SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS;         
  hSR_DelayRising2 = SYNCH_RECT_DELAY_RISING2_INIT_HR_TICKS;           
  hSR_DelayFalling2 = SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS;
#endif
  
  /* set PWM period at init value */
  hPWMPeriod = HRTIM_INIT_PWM_PERIOD;
  
  /* Reset update frequency counter */
  hStartUpFreqUpdateCounter = START_UP_COUNTER_PRESCALER_RATIO;
  
#ifdef FAN_PWM_DRIVING
  FAN_PWMDutyCycle = FAN_PWM_INIT_DUTY;
#endif

  /* update LLC period and SR registers to prevent a switching on with an old value */
  
#ifdef SYNCH_RECTIFICATION
    /* Disable Update of Master timer and all timers used */
    HAL_HRTIM_UpdateDisable(&hhrtim, (HRTIM_TIMERUPDATE_MASTER | PWM_FB_HS1_LS1_HRTIM_TIMERUPDATE | PWM_FB_HS2_LS2_HRTIM_TIMERUPDATE | PWM_SR_HS2_LS1_HRTIM_TIMERUPDATE | PWM_SR_HS1_LS2_HRTIM_TIMERUPDATE));
//    /* update PWM period */
//    PWM_PeriodActuation(&hhrtim, hPWMPeriod);
//    /* update SR waveforms */
//    PWM_SynchRectActuation(&hhrtim, hPWMPeriod, hSR_DelayRising1, hSR_DelayFalling1, hSR_DelayRising2, hSR_DelayFalling2);
    /* actuation of main and SR PWM signals with a variable dead time */
    PWM_PeriodSynchRectActuationDT(&hhrtim, hPWMPeriod, hSR_DelayRising1, hSR_DelayFalling1, hSR_DelayRising2, hSR_DelayFalling2, hPWMDeadTimeHRticks);
    /* Enable Update of Master timer and all timers used - in this manner all registers are updated together at next MASTER repetition event */
    HAL_HRTIM_UpdateEnable(&hhrtim, (HRTIM_TIMERUPDATE_MASTER | PWM_FB_HS1_LS1_HRTIM_TIMERUPDATE | PWM_FB_HS2_LS2_HRTIM_TIMERUPDATE | PWM_SR_HS2_LS1_HRTIM_TIMERUPDATE | PWM_SR_HS1_LS2_HRTIM_TIMERUPDATE));
#else
    /* update PWM period - only MASTER period is changend in half mode: it is not necessary disable/enable the update of registers */
    PWM_PeriodActuation(&hhrtim, hPWMPeriod);
#endif
  
#ifdef USE_EXTENDED_PID
  /* reset PI integral terms for Vout regulator */
  PID_Reset_IntegralTerm_ex(&PID_Vout_InitStructure);
#else
  /* reset PI integral terms for Vout regulator */
  PID_Reset_IntegralTerm(&PID_Vout_InitStructure);
#endif
  
  /* Init configuration param struct ---------------------------------------- */
  
  /* Restore init values for SR, dead time and frequency */
  DCDC_ConfigParamStruct.wOpenLoopFreq_Hz = HRTIM_MAX_PWM_FREQ_HZ;
  DCDC_ConfigParamStruct.hFixedSRDelayRising1_ns = SYNCH_RECT_DELAY_RISING1_INIT_NS;
  DCDC_ConfigParamStruct.hFixedSRDelayFalling1_ns = SYNCH_RECT_DELAY_FALLING1_INIT_NS;
  DCDC_ConfigParamStruct.hFixedSRDelayRising2_ns = SYNCH_RECT_DELAY_RISING2_INIT_NS;
  DCDC_ConfigParamStruct.hFixedSRDelayFalling2_ns = SYNCH_RECT_DELAY_FALLING2_INIT_NS;
  DCDC_ConfigParamStruct.hDeadTimeFullBridge_ns = DEAD_TIME_RISING_NS;
//  DCDC_ConfigParamStruct.hRegulatorKpGain = PID_VOUT_KP_DEFAULT;      // controller gains not updated with default values
//  DCDC_ConfigParamStruct.hRegulatorKiGain = PID_VOUT_KI_DEFAULT;      // controller gains not updated with default values
  
  DCDC_ConfigParamStruct.bConfigurationChanged = TRUE; //update init configuration
  /* ------------------------------------------------------------------------ */
  
  /* set fault LED off */
  LED_SetParams(&FaultLED, LED_OFF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
}

/**
  * @brief  This function sets the HRTIM period to have the desired PWM frequency
  * @param  wPWMFreqHz PWM Frequency in Hz
  * @retval PWM period in HRTIM ticks 
  *
  */
uint16_t CTR_SetPWMFrequency(uint32_t wPWMFreqHz)
{
  if(wPWMFreqHz > HRTIM_MAX_PWM_FREQ_START_UP_HZ){
    wPWMFreqHz = HRTIM_MAX_PWM_FREQ_START_UP_HZ;
  }
  else if(wPWMFreqHz < HRTIM_MIN_PWM_FREQ_HZ){
    wPWMFreqHz = HRTIM_MIN_PWM_FREQ_HZ;
  }
  
  /* return PWM period variable */
  return (FREQ_HZ_TO_PWM_TICKS(wPWMFreqHz)); 
}

/**
  * @brief  This function sets the HRTIM dead time variables to the desired value in ns
  * @param  hDeadTime_ns: dead time in ns
  * @retval None
  *
  */
void CTR_SetPWMDeadTime(uint16_t hPWMDeadTime_ns)
{
  if(hPWMDeadTime_ns > DEAD_TIME_MAX_NS){
    hPWMDeadTime_ns = DEAD_TIME_MAX_NS;
  }
  else if(hPWMDeadTime_ns < DEAD_TIME_MIN_NS){
    hPWMDeadTime_ns = DEAD_TIME_MIN_NS;
  }
  
  /* the dead time value in DT ticks format */
  hPWMDeadTime = DEAD_TIME_NS_2_DT_TICKS(hPWMDeadTime_ns);
  
  /* the dead time value in HRTIM ticks (used to calc the SR rising and falling edges) */
  hPWMDeadTimeHRticks = DELAY_NS_2_HRTIM_TICKS(hPWMDeadTime_ns);
  
  /* set dead time changed flag */
  bDCDC_DeadTimeChanged = TRUE;  
}

/**
  * @brief  This function sets the HRTIM rising/falling edges variables to have the desired delays in ns
  * @param  hSR_DelayRising1_ns: rising edge delay for SR signal 1
  * @param  hSR_DelayFalling1_ns: falling edge delay for SR signal 1
  * @param  hSR_DelayRising2_ns: rising edge delay for SR signal 2
  * @param  hSR_DelayFalling2_ns: falling edge delay for SR signal 2
  * @retval None 
  *
  */
void CTR_SetSRFixedDelays(DCDC_ConfigParamStruct_t* pConfigParamStruct)
{
  int16_t hSR_DelayRising1_ns = pConfigParamStruct->hFixedSRDelayRising1_ns;
  int16_t hSR_DelayFalling1_ns = pConfigParamStruct->hFixedSRDelayFalling1_ns;
  int16_t hSR_DelayRising2_ns = pConfigParamStruct->hFixedSRDelayRising2_ns;
  int16_t hSR_DelayFalling2_ns = pConfigParamStruct->hFixedSRDelayFalling2_ns;
  
  /* hSR_DelayRising1_ns boundary limitation */
  if(hSR_DelayRising1_ns > SYNCH_RECT_DELAY_RISING1_MAX_NS){
    hSR_DelayRising1_ns = SYNCH_RECT_DELAY_RISING1_MAX_NS;
  }
  else if(hSR_DelayRising1_ns < SYNCH_RECT_DELAY_RISING1_MIN_NS){
    hSR_DelayRising1_ns = SYNCH_RECT_DELAY_RISING1_MIN_NS;
  }
  
  /* hSR_DelayFalling1_ns boundary limitation */
  if(hSR_DelayFalling1_ns > SYNCH_RECT_DELAY_FALLING1_MAX_NS){
    hSR_DelayFalling1_ns = SYNCH_RECT_DELAY_FALLING1_MAX_NS;
  }
  else if(hSR_DelayFalling1_ns < SYNCH_RECT_DELAY_FALLING1_MIN_NS){
    hSR_DelayFalling1_ns = SYNCH_RECT_DELAY_FALLING1_MIN_NS;
  }
  
  /* hSR_DelayRising2_ns boundary limitation */
  if(hSR_DelayRising2_ns > SYNCH_RECT_DELAY_RISING2_MAX_NS){
    hSR_DelayRising2_ns = SYNCH_RECT_DELAY_RISING2_MAX_NS;
  }
  else if(hSR_DelayRising2_ns < SYNCH_RECT_DELAY_RISING2_MIN_NS){
    hSR_DelayRising2_ns = SYNCH_RECT_DELAY_RISING2_MIN_NS;
  }
  
  /* hSR_DelayFalling2_ns boundary limitation */
  if(hSR_DelayFalling2_ns > SYNCH_RECT_DELAY_FALLING2_MAX_NS){
    hSR_DelayFalling2_ns = SYNCH_RECT_DELAY_FALLING2_MAX_NS;
  }
  else if(hSR_DelayFalling2_ns < SYNCH_RECT_DELAY_FALLING2_MIN_NS){
    hSR_DelayFalling2_ns = SYNCH_RECT_DELAY_FALLING2_MIN_NS;
  }
  
  /* update SR delay variables */
  hSR_DelayRising1 = DELAY_NS_2_HRTIM_TICKS_SIGNED(hSR_DelayRising1_ns);
  hSR_DelayFalling1 = DELAY_NS_2_HRTIM_TICKS_SIGNED(hSR_DelayFalling1_ns);
  hSR_DelayRising2 = DELAY_NS_2_HRTIM_TICKS_SIGNED(hSR_DelayRising2_ns);  
  hSR_DelayFalling2 = DELAY_NS_2_HRTIM_TICKS_SIGNED(hSR_DelayFalling2_ns);  
}

/**
  * @brief  This function updates the configuration parameters of the converter
  * @param  pConfigParamStruct: pointer to configuration parameters struct
  * @param  hhrtim: pointer to HRTIM handle
  * @retval None 
  */
void CTR_UpdateConfigParam(DCDC_ConfigParamStruct_t* pConfigParamStruct, HRTIM_HandleTypeDef * hhrtim)
{
  if(pConfigParamStruct->bConfigurationChanged == TRUE){
    
    /* output enable/disable command */
    if(pConfigParamStruct->bConverterEnabled == TRUE)
    {
      STM_SetConverterEnabledFag(TRUE);
    }
    else{
      /* when the converter is disabled, also the SR is disabled - only if AUTOMATIC_SR_TURN_ON_OFF is not defined */
#ifndef AUTOMATIC_SR_TURN_ON_OFF
      pConfigParamStruct->bSREnabled = FALSE;
#endif
      STM_SetConverterEnabledFag(FALSE);
      STM_SetStateMachineStatus(DSMPS_STOP);      
    }
    
    /* adaptive SR enable/disable */
    if(pConfigParamStruct->bAdaptiveSREnabled == TRUE)
    {
      CTR_AdaptiveSynchRectEnable();
    }
    else
    {
      CTR_AdaptiveSynchRectDisable();
    }
    
    /* ligth load burst mode enable/disable */
    if(pConfigParamStruct->bBurstModeEnabled == TRUE)
    {
      // The burst mode is automatically turned on/off depending on load current 
    }
    else // burst mode disabled
    {
      /* call burst mode control fnc with a current greather then the threshold needed to set BMRange at BM_NOBURST value */
//      CTR_BurstModeControl(BURST_MODE_IOUT_RANGE3_TH_H + 1, hhrtim); // 3 BURST THRESHOLDS
      CTR_BurstModeControl(BURST_MODE_IOUT_RANGE23_TH_H + 1, hhrtim); // 2 BURST THRESHOLDS
//      CTR_BurstModeControl(BURST_MODE_IOUT_RANGE12_TH_H + 1, hhrtim); // 1 BURST THRESHOLDS
      //integral term is set equal to last PWM period multiplied by ki divisor, then if error_k = 0 the f_closed_loop_k =  f_open_loop_k-1 */
      CTR_UpdateRegulatorOutput(); 
      /* disable burst mode */
      CTR_BurstModeDisable(hhrtim);
    }
    
/* enable immediately only if AUTOMATIC_SR_TURN_ON_OFF is not defined, otherwise turn-on/off depends on Iout together with the enable flag */
    /* only in run state SR can be enabled/disabled */
    if(STM_GetStateMachineStatus() == DSMPS_RUN){
      /* SR enable/disable command */
      if(pConfigParamStruct->bSREnabled == TRUE)
      {
#ifndef AUTOMATIC_SR_TURN_ON_OFF            
        /* enable gate drivers for SR PWMs */
        CTR_SRGateDriverEnable();
        /* enable SR outputs */
        CTR_PWMSynchRectOutputEnable(hhrtim);
#endif
      }
      else{
        /* disable SR outputs */
        CTR_PWMSynchRectOutputDisable(hhrtim);
        /* disable gate drivers for SR PWMs */
        CTR_SRGateDriverDisable();      
      }
    }
    
    /* enable/disable fan */
    if(pConfigParamStruct->bFanPWMDrivingEnabled == TRUE){
      CTR_FanEnable();
    }
    else{ 
      CTR_FanDisable();
    }
    
    if(DCDC_ConfigParamStruct.bOpenLoopEnabled == TRUE)
    {
      /* disable output undervoltage check if open loop is enabled */
      bCheckUndervoltageEnabled = FALSE;
    }
    else{
      /* update of regulator output when the control loop is closed */
//      CTR_UpdateRegulatorOutput();
      /* disable output undervoltage check if open loop is enabled */
//      bCheckUndervoltageEnabled = TRUE;
    }
    
    /* change control params */
#ifdef USE_EXTENDED_PID
    PID_Set_ProportionalGain_ex(&PID_Vout_InitStructure, pConfigParamStruct->hRegulatorKpGain);
    PID_Set_IntegralGain_ex(&PID_Vout_InitStructure, pConfigParamStruct->hRegulatorKiGain);
#else
    PID_Set_ProportionalGain(&PID_Vout_InitStructure, pConfigParamStruct->hRegulatorKpGain);
    PID_Set_IntegralGain(&PID_Vout_InitStructure, pConfigParamStruct->hRegulatorKiGain);
#endif
    
//    hPWMPeriod = CTR_SetPWMFrequency(pConfigParamStruct->wOpenLoopFreq_Hz); // open loop frequency already set in control IrqHandler
    CTR_SetPWMDeadTime(pConfigParamStruct->hDeadTimeFullBridge_ns);
    CTR_SetSRFixedDelays(pConfigParamStruct);
    pConfigParamStruct->bConfigurationChanged = FALSE; // comment to latch the changing of configuration params
  }  
}
/**
  * @brief  Calculates switching frequency to control Vout
  * @param  hVoltageReference output Voltage reference 
  * @param  hVoltageMeasure actual output Voltage measure
  * @retval PWM period value
*/
#pragma location = ".ccmram"
uint16_t CTR_ExecuteVoltageLoop(uint16_t hVoltageReference, uint16_t hVoltageMeasure)              
{
  uint16_t hPWMPeriod_tmp = 0;
  
#ifdef USE_EXTENDED_PID
  hPWMPeriod_tmp = PID_Regulator_ex(hVoltageReference, hVoltageMeasure, &PID_Vout_InitStructure);
#else
  hPWMPeriod_tmp = PID_Regulator(hVoltageReference, hVoltageMeasure, &PID_Vout_InitStructure);
#endif
  
  return hPWMPeriod_tmp;
}

/**
  * @brief  This function calculates SR falling edges in adaptive SR
  * @param  None
  * @retval None
  *
  * The Fallig edges of SR PWMs are adjusted comparing the Vds measures (acquired after MOSFET's turn-off) with the corresponding thresholds:
  * if the measure is below the thresholds, it means that the body diode is still conducting when the Vds is acquired, than the falling edge
  * delayed (delay respect to primary PWMs reduced), otherwise the diode is not conducting, so the the falling edge is anticipated (delay 
  * respect to primary PWMs increased). In any case the PWMs are stoped if the comparators are triggered for an immediatly shut-down.
  * 
  */
#pragma location = ".ccmram"
void CTR_AdaptiveSynchRectEdgeCalculation(void)
{  
  /* load global variables */
  int16_t hSR_DelayFalling_tmp1 = hSR_DelayFalling1;
  int16_t hSR_DelayFalling_tmp2 = hSR_DelayFalling2;
  
  /* debug ADC injected measures */
  DCDC_MeasureStruct.hVdsSR1 = PWM_GetVdsSRMeasure(SR_LEG1);
  DCDC_MeasureStruct.hVdsSR2 = PWM_GetVdsSRMeasure(SR_LEG2);
    
  /* SR1 adjusting falling delay */
  if(DCDC_MeasureStruct.hVdsSR1 > ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH2){
    hSR_DelayFalling_tmp1 +=ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY2_HR_TICKS;
    /* check upper limit value */
    if(hSR_DelayFalling_tmp1 > SYNCH_RECT_DELAY_FALLING1_MAX_HR_TICKS){
      hSR_DelayFalling_tmp1 = SYNCH_RECT_DELAY_FALLING1_MAX_HR_TICKS;
    }
  }
  else if(DCDC_MeasureStruct.hVdsSR1 > ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH1){
    hSR_DelayFalling_tmp1 +=ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY1_HR_TICKS;
    /* check upper limit value */
    if(hSR_DelayFalling_tmp1 > SYNCH_RECT_DELAY_FALLING1_MAX_HR_TICKS){
      hSR_DelayFalling_tmp1 = SYNCH_RECT_DELAY_FALLING1_MAX_HR_TICKS;
    }
  }
  else{
    hSR_DelayFalling_tmp1 -=ADAPTIVE_SYNCH_RECT_DECREMENTAL_DELAY_HR_TICKS;
    /* check lower limit value */
    if(hSR_DelayFalling_tmp1 < SYNCH_RECT_DELAY_FALLING1_MIN_HR_TICKS){
      hSR_DelayFalling_tmp1 = SYNCH_RECT_DELAY_FALLING1_MIN_HR_TICKS;
    }
  }
  
  /* SR2 adjusting falling delay */
  if(DCDC_MeasureStruct.hVdsSR2 > ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH2){
    hSR_DelayFalling_tmp2 +=ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY2_HR_TICKS;
    /* check upper limit value */
    if(hSR_DelayFalling_tmp2 > SYNCH_RECT_DELAY_FALLING2_MAX_HR_TICKS){
      hSR_DelayFalling_tmp2 = SYNCH_RECT_DELAY_FALLING2_MAX_HR_TICKS;
    }
  }
  else if(DCDC_MeasureStruct.hVdsSR2 > ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH1){
    hSR_DelayFalling_tmp2 +=ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY1_HR_TICKS;
    /* check upper limit value */
    if(hSR_DelayFalling_tmp2 > SYNCH_RECT_DELAY_FALLING2_MAX_HR_TICKS){
      hSR_DelayFalling_tmp2 = SYNCH_RECT_DELAY_FALLING2_MAX_HR_TICKS;
    }
  }
  else{
    hSR_DelayFalling_tmp2 -=ADAPTIVE_SYNCH_RECT_DECREMENTAL_DELAY_HR_TICKS;
    /* check lower limit value */
    if(hSR_DelayFalling_tmp2 < SYNCH_RECT_DELAY_FALLING2_MIN_HR_TICKS){
      hSR_DelayFalling_tmp2 = SYNCH_RECT_DELAY_FALLING2_MIN_HR_TICKS;
    }
  }
  
  /* update global variables */
  hSR_DelayFalling1 = hSR_DelayFalling_tmp1;
  hSR_DelayFalling2 = hSR_DelayFalling_tmp2;
    
}

/**
  * @brief  Execute control logic for burst mode actuation at light load - TWO burst mode levels
  * @param  hOuputCurrent output current measure
  * @param  hhrtim: pointer to HRTIM handle
  * @retval none
*/
#pragma location = ".ccmram"
void CTR_BurstModeControl(uint16_t hOuputCurrent, HRTIM_HandleTypeDef * hhrtim)
{  
#define BURST_MODE_VALIDATION_NUM 3
  static DCDC_BMRange_t BMRange = BM_NOBURST;   /* static burst mode range variable */
  DCDC_BMRange_t BMRange_tmp = BM_NOBURST;      /* new burst mode range variable */
  static uint8_t nBMValidationCounter = BURST_MODE_VALIDATION_NUM;
//  /* burst mode range variable calculation - three levels *********************/
//  if((hOuputCurrent > BURST_MODE_IOUT_RANGE3_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE3_TH_L)&&(BMRange == BM_NOBURST)))
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_NOBURST;
//  }
//  else if((hOuputCurrent > BURST_MODE_IOUT_RANGE23_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE23_TH_L)&&(BMRange == BM_RANGE3)))
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_RANGE3;
//  }
//  else if((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_L)&&(BMRange == BM_RANGE2)))
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_RANGE2;
//  }
//  else // range1
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_RANGE1;
//  }
  
  /* burst mode range variable calculation - two levels ***********************/
//  if((hOuputCurrent > BURST_MODE_IOUT_RANGE23_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE23_TH_L)&&(BMRange == BM_NOBURST)))
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_NOBURST;
//  }
//  else if((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_L)&&(BMRange == BM_RANGE2)))
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_RANGE2;
//  }
//  else // range1
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_RANGE1;
//  }
  /*************************************************************************** WORKING */
//#ifdef OUT_CURRENT_SENSOR_CALIBRATION
//  /* burst mode range variable calculation - one level with calibrated offset ***********************/
//  if((hOuputCurrent > hBmIoutRange12ThresholdHigh) || ((hOuputCurrent > hBmIoutRange12ThresholdLow)&&(BMRange == BM_RANGE1)))
//#else
//  /* burst mode range variable calculation - one level ***********************/
//  if((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_L)&&(BMRange == BM_RANGE1)))
//#endif
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_NOBURST;
//  }
//  else // range1
//  {
//    /* set burst mode range tmp variable */
//    BMRange_tmp = BM_RANGE1;
//  }  
  /****************************************************************************/
  
#ifdef OUT_CURRENT_SENSOR_CALIBRATION
  /* burst mode range variable calculation - two levels ***********************/
  if((hOuputCurrent > hBmIoutRange23ThresholdHigh) || ((hOuputCurrent > hBmIoutRange23ThresholdLow)&&(BMRange == BM_NOBURST)))
  {
    /* set burst mode range tmp variable */
    BMRange_tmp = BM_NOBURST;
  }
  else if((hOuputCurrent > hBmIoutRange12ThresholdHigh) || ((hOuputCurrent > hBmIoutRange12ThresholdLow)&&(BMRange == BM_RANGE2)))
  {
    /* set burst mode range tmp variable */
    BMRange_tmp = BM_RANGE2;
  }
  else // range1
  {
    /* set burst mode range tmp variable */
    BMRange_tmp = BM_RANGE1;
  } 
#else
  /* burst mode range variable calculation - two levels ***********************/
  if((hOuputCurrent > BURST_MODE_IOUT_RANGE23_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE23_TH_L)&&(BMRange == BM_NOBURST)))
  {
    /* set burst mode range tmp variable */
    BMRange_tmp = BM_NOBURST;
  }
  else if((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_H) || ((hOuputCurrent > BURST_MODE_IOUT_RANGE12_TH_L)&&(BMRange == BM_RANGE2)))
  {
    /* set burst mode range tmp variable */
    BMRange_tmp = BM_RANGE2;
  }
  else // range1
  {
    /* set burst mode range tmp variable */
    BMRange_tmp = BM_RANGE1;
  } 
#endif
  /****************************************************************************/
  
  /* disable burst mode if SR is enabled */  
  /* update static variable and burst mode idle/period */
  if(BMRange_tmp != BMRange){
    switch(BMRange_tmp){
    default:
    case BM_NOBURST:
    case BM_RANGE2:
    case BM_RANGE3:
      /* disable burst mode */
      CTR_BurstModeDisable(hhrtim);
      nBMValidationCounter = BURST_MODE_VALIDATION_NUM;
      break;
//    case BM_RANGE3:
//      if(bDCDC_SynchRectOutputEnabled == TRUE){
//        /* Disable output SR PWMs when burst mode is active */
//        CTR_PWMSynchRectOutputDisable(hhrtim);
//      }
//      /* update burst mode parameters */
//      PWM_UpdateBurstModeParams(hhrtim, BURST_MODE_PERIOD_RANGE3, BURST_MODE_IDLE_DURATION_RANGE3);
//      /* enable burst mode */
//      HAL_HRTIM_BurstModeCtl(hhrtim, HRTIM_BURSTMODECTL_ENABLED);
//      /* set Burst mode enable flag */
//      bLightLoadBurstModeEnabled = TRUE;
//      break; 
//    case BM_RANGE2:
//      if(bDCDC_SynchRectOutputEnabled == TRUE){
//        /* Disable output SR PWMs when burst mode is active */
//        CTR_PWMSynchRectOutputDisable(hhrtim);
//      }
//      /* update burst mode parameters */
//      PWM_UpdateBurstModeParams(hhrtim, BURST_MODE_PERIOD_RANGE2, BURST_MODE_IDLE_DURATION_RANGE2);
//      /* enable burst mode */
//      CTR_BurstModeEnable(hhrtim);
//      break; 
    case BM_RANGE1:
      {
//    case BM_RANGE2:
//    case BM_RANGE3:
//        if(--nBMValidationCounter == 0)
//        {
          if(bDCDC_SynchRectOutputEnabled == TRUE){
            /* Disable output SR PWMs when burst mode is active */
            CTR_PWMSynchRectOutputDisable(hhrtim);
          }
          /* update burst mode parameters */
          PWM_UpdateBurstModeParams(hhrtim, BURST_MODE_PERIOD_RANGE1, BURST_MODE_IDLE_DURATION_RANGE1);
          if(bVoutBurstModeEnabled == FALSE)
          {
            /* enable burst mode */
            CTR_BurstModeEnable(hhrtim);
          }
//          nBMValidationCounter = BURST_MODE_VALIDATION_NUM;          
//        }
      }
      break;
    }
    /* update static variable and burst mode idle/period */
    BMRange = BMRange_tmp;
  }

}

/**
  * @brief  Sets the counter passed by reference at new value
  * @param  phCounter_to_set: Pointer to the counter to set
  * @param  hNew_counter_value new counter value
  * @retval None
  */
#pragma location = ".ccmram"
void SetDelayTime(uint16_t volatile *phCounter_to_set, uint16_t hNew_counter_value)
{
 *phCounter_to_set = hNew_counter_value;
} 

/**
  * @brief  Checks if the counter passed by reference is equal to zero
  * @param  phCounter_to_check: Pointer to the counter to check
  * @retval bool TRUE or FALSE
  */
#pragma location = ".ccmram"
bool DelayTimeIsElapsed(uint16_t volatile *phCounter_to_check)
{
 if (*phCounter_to_check == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
}


/**
  * @brief  Enable Gate driver for Full Bridge devices.
  * @param  None
  * @retval None
  */
void CTR_GateDriverEnable(void)
{
  /* Enable pins of Full bridge LLC drivers are pull-upped */
}

/**
  * @brief  Disable Gate driver for Full Bridge devices.
  * @param  None
  * @retval None
  */
void CTR_GateDriverDisable(void)
{
  /* Enable pins of Full bridge LLC drivers are pull-upped */ 
}

/**
  * @brief  Charge boot capacitors of gate drivers changing the idle polarity of the low side PWMs
  * @param  None
  * @retval None
  */
void CTR_GateDriverBootstrapCapPrecharge(HRTIM_HandleTypeDef * hhrtim)
{ 
  static uint8_t nPrechargeCounterDuration = 0;
  
  if(FullBridgeDriverPrechargeStruct.prechargeState == PRECHARGE_OFF){
    /* Disable the counters */
    hhrtim->Instance->sMasterRegs.MCR &= ~(PWM_FB_HS1_LS1_HRTIM_TIMERID | PWM_FB_HS2_LS2_HRTIM_TIMERID | PWM_SR_HS2_LS1_HRTIM_TIMERID | PWM_SR_HS1_LS2_HRTIM_TIMERID | HRTIM_TIMERID_MASTER);

    /* set active IDLE state of low side outputs */
    /* get output configuration of HS1_LS1 TIM index */
    uint32_t hrtim_hs1_ls1_outr = hhrtim->Instance->sTimerxRegs[PWM_FB_HS1_LS1_HRTIM_TIMERINDEX].OUTxR;

    /* get output configuration of HS2_LS2 TIM index */
    uint32_t hrtim_hs2_ls2_outr = hhrtim->Instance->sTimerxRegs[PWM_FB_HS2_LS2_HRTIM_TIMERINDEX].OUTxR;

    /* Set active the IDLE state - NOTE: it is intented that low side output is output 2, then shift = 16, otherwise the shift is zero */
    hrtim_hs1_ls1_outr |= (HRTIM_OUTPUTIDLELEVEL_ACTIVE << 16);

    /* Set active the IDLE state - NOTE: it is intented that low side output is output 2, then shift = 16, otherwise the shift is zero */
    hrtim_hs2_ls2_outr |= (HRTIM_OUTPUTIDLELEVEL_ACTIVE << 16);

    /* Set output configuration register of HS1_LS1 TIM index */
    hhrtim->Instance->sTimerxRegs[PWM_FB_HS1_LS1_HRTIM_TIMERINDEX].OUTxR = hrtim_hs1_ls1_outr;

    /* Set output configuration register of HS2_LS2 TIM index */
    hhrtim->Instance->sTimerxRegs[PWM_FB_HS2_LS2_HRTIM_TIMERINDEX].OUTxR = hrtim_hs2_ls2_outr;
    
    nPrechargeCounterDuration = 100; // number of Vout control periods for precharge phase
    
    /* set next precharge state */
    FullBridgeDriverPrechargeStruct.prechargeState = PRECHARGE_ONGOING;
  }
  else if(FullBridgeDriverPrechargeStruct.prechargeState == PRECHARGE_ONGOING){
    nPrechargeCounterDuration--;
    if(nPrechargeCounterDuration == 0){
      /* set inactive IDLE state of low side outputs */
      /* get output configuration of HS1_LS1 TIM index */
      uint32_t hrtim_hs1_ls1_outr = hhrtim->Instance->sTimerxRegs[PWM_FB_HS1_LS1_HRTIM_TIMERINDEX].OUTxR;

      /* get output configuration of HS2_LS2 TIM index */
      uint32_t hrtim_hs2_ls2_outr = hhrtim->Instance->sTimerxRegs[PWM_FB_HS2_LS2_HRTIM_TIMERINDEX].OUTxR;

      /* Set active the IDLE state - NOTE: it is intented that low side output is output 2, then shift = 16, otherwise the shift is zero */
      hrtim_hs1_ls1_outr &= (~(HRTIM_OUTPUTIDLELEVEL_ACTIVE << 16));

      /* Set active the IDLE state - NOTE: it is intented that low side output is output 2, then shift = 16, otherwise the shift is zero */
      hrtim_hs2_ls2_outr &= (~(HRTIM_OUTPUTIDLELEVEL_ACTIVE << 16)); 

      /* Set output configuration register of HS1_LS1 TIM index */
      hhrtim->Instance->sTimerxRegs[PWM_FB_HS1_LS1_HRTIM_TIMERINDEX].OUTxR = hrtim_hs1_ls1_outr;

      /* Set output configuration register of HS2_LS2 TIM index */
      hhrtim->Instance->sTimerxRegs[PWM_FB_HS2_LS2_HRTIM_TIMERINDEX].OUTxR = hrtim_hs2_ls2_outr;
      
      /* enable the counters */
      hhrtim->Instance->sMasterRegs.MCR |= (PWM_FB_HS1_LS1_HRTIM_TIMERID | PWM_FB_HS2_LS2_HRTIM_TIMERID | PWM_SR_HS2_LS1_HRTIM_TIMERID | PWM_SR_HS1_LS2_HRTIM_TIMERID | HRTIM_TIMERID_MASTER);

      /* set next precharge state */
      FullBridgeDriverPrechargeStruct.prechargeState = PRECHARGE_HOLD;
    }    
  }
  else{ // FullBridgeDriverPrechargeStruct.prechargeState == PRECHARGE_HOLD
    /* reset enable precharge variable */
    FullBridgeDriverPrechargeStruct.bPrechargeEnabled = FALSE;
    /* set completed precharge variable */
    FullBridgeDriverPrechargeStruct.bPrechargeCompleted = TRUE;
    /* set next precharge state */
    FullBridgeDriverPrechargeStruct.prechargeState = PRECHARGE_OFF;
  }  
}


/**
  * @brief  Enable main PWMs with bootstrap caps precharge, if defined
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_PWMOutputEnable(HRTIM_HandleTypeDef * hhrtim)
{
    
#ifdef GATE_DRIVER_BOOTSTRAP_PRECHARGE
  /* set precharge enable variable */
  FullBridgeDriverPrechargeStruct.bPrechargeEnabled = TRUE;
  
  /* wait until bootstrap capacitors are charged */
  while(FullBridgeDriverPrechargeStruct.bPrechargeCompleted != TRUE);
  
  /* reset precharge complete variable for next precharge phase */
  FullBridgeDriverPrechargeStruct.bPrechargeCompleted = FALSE;
#endif
  
  /* update PWM period and registers for ADC acquisitions */
  PWM_PeriodActuation(hhrtim, hPWMPeriod);
  
  /* Force software update */
  HAL_HRTIM_SoftwareUpdate(hhrtim, HRTIM_TIMERUPDATE_MASTER);
    
  /* Main Output Enable */
  HAL_HRTIM_WaveformOutputStart(hhrtim, PWM_FB_HS1_HRTIM_OUTPUT | PWM_FB_LS1_HRTIM_OUTPUT | PWM_FB_HS2_HRTIM_OUTPUT | PWM_FB_LS2_HRTIM_OUTPUT);   

  /* set boolean variable */
  bDCDC_OutputEnabled = TRUE;
}

/**
  * @brief  Fast enable of main PWMs without bootstrap caps precharge
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_PWMFastOutputEnable(HRTIM_HandleTypeDef * hhrtim)
{  
  /* update PWM period and registers for ADC acquisitions */
  PWM_PeriodActuation(hhrtim, hPWMPeriod);
  
  /* Force software update */
  HAL_HRTIM_SoftwareUpdate(hhrtim, HRTIM_TIMERUPDATE_MASTER);
    
  /* Main Output Enable */
  HAL_HRTIM_WaveformOutputStart(hhrtim, PWM_FB_HS1_HRTIM_OUTPUT | PWM_FB_LS1_HRTIM_OUTPUT | PWM_FB_HS2_HRTIM_OUTPUT | PWM_FB_LS2_HRTIM_OUTPUT);   

  /* set boolean variable */
  bDCDC_OutputEnabled = TRUE;
}

/**
  * @brief  Disable main PWMs.
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_PWMOutputDisable(HRTIM_HandleTypeDef * hhrtim)
{    
  /* Output Disable */
  HAL_HRTIM_WaveformOutputStop(hhrtim, PWM_FB_HS1_HRTIM_OUTPUT | PWM_FB_LS1_HRTIM_OUTPUT | PWM_FB_HS2_HRTIM_OUTPUT | PWM_FB_LS2_HRTIM_OUTPUT);  
  
  /* reset boolean variable */
  bDCDC_OutputEnabled = FALSE; 
}

/**
  * @brief  Disable both main PWMs and synch. rect. simultaneously
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_PWMAllOutputsDisable(HRTIM_HandleTypeDef * hhrtim)
{    
  /* Output Disable */
  HAL_HRTIM_WaveformOutputStop(hhrtim, PWM_FB_HS1_HRTIM_OUTPUT | PWM_FB_LS1_HRTIM_OUTPUT | PWM_FB_HS2_HRTIM_OUTPUT | PWM_FB_LS2_HRTIM_OUTPUT | PWM_SR_HS1_HRTIM_OUTPUT | PWM_SR_LS1_HRTIM_OUTPUT | PWM_SR_HS2_HRTIM_OUTPUT | PWM_SR_LS2_HRTIM_OUTPUT);  
  
  if(bDCDC_AdaptiveSynchRectEnabled == TRUE){
    /* restore SR init values */
    hSR_DelayFalling1 = SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS;         
    hSR_DelayFalling2 = SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS;
  }
 
  /* reset boolean variable */
  bDCDC_OutputEnabled = FALSE;
  
  /* reset boolean variable */
  bDCDC_SynchRectOutputEnabled = FALSE;
}

/**
  * @brief  Enable PWMs of Synchronous rectification
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_PWMSynchRectOutputEnable(HRTIM_HandleTypeDef * hhrtim)
{         
  /* Synch Rect output enable */
  HAL_HRTIM_WaveformOutputStart(hhrtim, PWM_SR_HS1_HRTIM_OUTPUT | PWM_SR_LS1_HRTIM_OUTPUT | PWM_SR_HS2_HRTIM_OUTPUT | PWM_SR_LS2_HRTIM_OUTPUT);
  
  /* set boolean variable */
  bDCDC_SynchRectOutputEnabled = TRUE;
}

/**
  * @brief  Disable PWMs of Synch. rect.
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_PWMSynchRectOutputDisable(HRTIM_HandleTypeDef * hhrtim)
{  
  /* Synch Rect output enable */
  HAL_HRTIM_WaveformOutputStop(hhrtim, PWM_SR_HS1_HRTIM_OUTPUT | PWM_SR_LS1_HRTIM_OUTPUT | PWM_SR_HS2_HRTIM_OUTPUT | PWM_SR_LS2_HRTIM_OUTPUT);
  
  if(bDCDC_AdaptiveSynchRectEnabled == TRUE){
    /* restore SR init values */
    hSR_DelayFalling1 = SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS;         
    hSR_DelayFalling2 = SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS;
  }
  
  /* reset boolean variable */
  bDCDC_SynchRectOutputEnabled = FALSE;
}

/**
  * @brief  Enable Adaptive SR:  enable COMPs and reset delays
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_AdaptiveSynchRectEnable(void)
{ 
  if(bDCDC_AdaptiveSynchRectEnabled == FALSE){
    /* enable COMP for adaptive SR */
    HAL_COMP_Start(&CompSRHandle1);
    HAL_COMP_Start(&CompSRHandle2);
    
    /* restore SR init values */
  //  hSR_DelayRising1 = SYNCH_RECT_DELAY_RISING1_INIT_HR_TICKS;           
    hSR_DelayFalling1 = SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS;         
  //  hSR_DelayRising2 = SYNCH_RECT_DELAY_RISING2_INIT_HR_TICKS;           
    hSR_DelayFalling2 = SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS;

    /* set Adaptive SR enabled flag */
    bDCDC_AdaptiveSynchRectEnabled = TRUE;  
  }
}

/**
  * @brief  Disable Adaptive SR:  disable COMPs
  * @param  hhrtim: pointer to HRTIM handler
  * @retval None
  */
void CTR_AdaptiveSynchRectDisable(void)
{  
  if(bDCDC_AdaptiveSynchRectEnabled == TRUE){  
    /* disable COMP for adaptive SR */
    HAL_COMP_Stop(&CompSRHandle1);
    HAL_COMP_Stop(&CompSRHandle2);
    
    /* reset Adaptive SR enabled flag */
    bDCDC_AdaptiveSynchRectEnabled = FALSE;  
  }
}

/**
  * @brief  Enable Gate driver for Synchronous Rectification devices.
  * @param  None
  * @retval None
  */
void CTR_SRGateDriverEnable(void)
{
  /* Enable Driver for SR Leg1 */
  HAL_GPIO_WritePin(SD_OD_SR1_GPIO_PORT, SD_OD_SR1_GPIO_PIN, GPIO_PIN_SET);
  
  /* Enable Driver for SR Leg2 */
  HAL_GPIO_WritePin(SD_OD_SR2_GPIO_PORT, SD_OD_SR2_GPIO_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Enable Gate driver for Synchronous Rectification devices.
  * @param  None
  * @retval None
  */
void CTR_SRGateDriverDisable(void)
{
  /* Disable Driver for SR Leg1 */
  HAL_GPIO_WritePin(SD_OD_SR1_GPIO_PORT, SD_OD_SR1_GPIO_PIN, GPIO_PIN_RESET);
  
  /* Disable Driver for SR Leg2 */
  HAL_GPIO_WritePin(SD_OD_SR2_GPIO_PORT, SD_OD_SR2_GPIO_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Enable Burst Mode.
  * @param  hhrtim: pointer to HRTIM handle
  * @retval None
  */
void CTR_BurstModeEnable(HRTIM_HandleTypeDef * hhrtim)
{
  /* enable burst mode */
  HAL_HRTIM_BurstModeCtl(hhrtim, HRTIM_BURSTMODECTL_ENABLED);
  /* set Burst mode enable flag */
  bLightLoadBurstModeEnabled = TRUE;
}
      
/**
  * @brief  Disable Burst Mode.
  * @param  hhrtim: pointer to HRTIM handle
  * @retval None
  */
void CTR_BurstModeDisable(HRTIM_HandleTypeDef * hhrtim)
{
  /* disable burst mode */
  HAL_HRTIM_BurstModeCtl(hhrtim, HRTIM_BURSTMODECTL_DISABLED);
  /* reset Burst mode enable flag */
  bLightLoadBurstModeEnabled = FALSE;
}

/**
  * @}
  */ 

/** @addtogroup FAN_Management
  * @{
  */

/**
  * @brief Disable Fan.
  * @param  None
  * @retval None
  */
void CTR_FanDisable(void)
{
#ifdef FAN_PWM_DRIVING
  HAL_TIM_PWM_Stop(&FanPwmHandle, FAN_PWM_TIM_CHANNEL);
#else
  HAL_GPIO_WritePin(FAN_PWM_GPIO_PORT, FAN_PWM_GPIO_PIN, GPIO_PIN_RESET);
#endif
  
  bFanEnabled = FALSE;
}

/**
  * @brief  Enable Fan.
  * @param  None
  * @retval None
  */
void CTR_FanEnable(void)
{
#ifdef FAN_PWM_DRIVING
  HAL_TIM_PWM_Start(&FanPwmHandle, FAN_PWM_TIM_CHANNEL);
#else
  HAL_GPIO_WritePin(FAN_PWM_GPIO_PORT, FAN_PWM_GPIO_PIN, GPIO_PIN_SET);
#endif
  
  bFanEnabled = TRUE;
}

/**
  * @brief  Regulation of Fan speed depending on temperature and output load
  * @param  fTemperature temperature of heatsink in °C
  * @param  hOutputCurrent actual value of output current
  * @param  hFantim: pointer to Fan TIM handler
  * @retval None
  * 
  * set FAN speed depending on load current and heat sink temperature
  *
  */
void CTR_FanSpeedRegulationTempLoad(float fTemperature, uint16_t hOutputCurrent, TIM_HandleTypeDef * hFantim)
{
//  static uint16_t FAN_PWMDutyCycle;
  static bool bFanMaxDuty = FALSE;
  uint16_t FAN_PWMDutyCycle_tmp;
   
  /* if the temperature is greater than FAN_TEMPERATURE_THRESHOLD (with hysteresis), the duty cycle of fan is set at maximum value */ 
  if ((fTemperature > FAN_TEMPERATURE_THRESHOLD_H)||((fTemperature > FAN_TEMPERATURE_THRESHOLD_L)&&(bFanMaxDuty == TRUE))){
    FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_HIGH_SPEED;
    bFanMaxDuty = TRUE;
  }
  else{
    bFanMaxDuty = FALSE;
  }
  
  /* if the temperature is lower than FAN_TEMPERATURE_THRESHOLD , the duty cycle of fan depends on load */
  if(bFanMaxDuty == FALSE){
    
    if(hOutputCurrent > FAN_HIGH_LOAD_THRESHOLD){
      FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_HIGH_SPEED;
    }
    else if(hOutputCurrent < FAN_LOW_LOAD_THRESHOLD){
      FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_LOW_SPEED;
    }
    else{ // linear relationship with load
      FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_LOW_SPEED + (int32_t)((FAN_ALPHA_DUTY_LOAD*(hOutputCurrent - FAN_LOW_LOAD_THRESHOLD))/1024);  
    }
  }
  
  /* increase/decrease actual duty cycle */
  if (FAN_PWMDutyCycle < (FAN_PWMDutyCycle_tmp - FAN_PWM_DELTA_DUTY)){
    FAN_PWMDutyCycle += FAN_PWM_DELTA_DUTY;
  }
  else if(FAN_PWMDutyCycle > (FAN_PWMDutyCycle_tmp + FAN_PWM_DELTA_DUTY)){
    FAN_PWMDutyCycle -= FAN_PWM_DELTA_DUTY;
  }
  
  /* update fan PWM duty cycle */
  PWM_FanActuation(hFantim, FAN_PWM_TIM_CHANNEL, FAN_PWMDutyCycle);
}

/**
  * @brief  Regulation of Fan speed depending on temperature and output load, the fan is disabled at very low load
  * pConfigParamStruct: pointer to configuration parameters struct
  * @param  fTemperature temperature of heatsink in °C
  * @param  hOutputCurrent actual value of output current
  * @param  hFantim: pointer to Fan TIM handler
  * @retval None
  * 
  * set FAN speed depending on load current and heat sink temperature
  *
  */
void CTR_FanSpeedRegulationTempLoadDis(DCDC_ConfigParamStruct_t* pConfigParamStruct, float fTemperature, uint16_t hOutputCurrent, TIM_HandleTypeDef * hFantim)
{
//  static uint16_t FAN_PWMDutyCycle; // local variable instead of global variable
  static bool bFanMaxDuty = FALSE;
  uint16_t FAN_PWMDutyCycle_tmp;
   
  /* if the temperature is greater than FAN_TEMPERATURE_THRESHOLD (with hysteresis), the duty cycle of fan is set at maximum value */ 
  if ((fTemperature > FAN_TEMPERATURE_THRESHOLD_H)||((fTemperature > FAN_TEMPERATURE_THRESHOLD_L)&&(bFanMaxDuty == TRUE))){
    FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_HIGH_SPEED;
    bFanMaxDuty = TRUE;
  }
  else{
    bFanMaxDuty = FALSE;
  }
  
  /* if the temperature is lower than FAN_TEMPERATURE_THRESHOLD , the duty cycle of fan depends on load */
  if(bFanMaxDuty == FALSE){
    
    /* disable fan for very low load *********/
//    if(pConfigParamStruct->bFanPWMDrivingEnabled == TRUE){
//      if((hOutputCurrent < FAN_DISABLE_LOAD_THRESHOLD)&&(bFanEnabled == TRUE)){
//        CTR_FanDisable();
//      }
//      else if((hOutputCurrent > FAN_ENABLE_LOAD_THRESHOLD)&&(bFanEnabled == FALSE)){ 
//        CTR_FanEnable();
//      }
//    }
    /****************************************/
    
    /* disable fan when light load burst mode is active *********/
      if((bLightLoadBurstModeEnabled == TRUE)&&(bFanEnabled == TRUE)){
        CTR_FanDisable();
      }
      else if((bLightLoadBurstModeEnabled == FALSE)&&(bFanEnabled == FALSE)){ 
        CTR_FanEnable();
      }

    /****************************************/
    
    if(hOutputCurrent > FAN_HIGH_LOAD_THRESHOLD){
      FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_HIGH_SPEED;
    }
    else if(hOutputCurrent < FAN_LOW_LOAD_THRESHOLD){
      FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_LOW_SPEED;
    }
    else{ // linear relationship with load
      FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_LOW_SPEED + (int32_t)((FAN_ALPHA_DUTY_LOAD*(hOutputCurrent - FAN_LOW_LOAD_THRESHOLD))/1024);  
    }
  }
  
  /* increase/decrease actual duty cycle */
  if (FAN_PWMDutyCycle < (FAN_PWMDutyCycle_tmp - FAN_PWM_DELTA_DUTY)){
    FAN_PWMDutyCycle += FAN_PWM_DELTA_DUTY;
  }
  else if(FAN_PWMDutyCycle > (FAN_PWMDutyCycle_tmp + FAN_PWM_DELTA_DUTY)){
    FAN_PWMDutyCycle -= FAN_PWM_DELTA_DUTY;
  }
  
  /* update fan PWM duty cycle */
  PWM_FanActuation(hFantim, FAN_PWM_TIM_CHANNEL, FAN_PWMDutyCycle);
}

/**
  * @brief  Regulation of Fan speed depending on phase shift
  * @param  hOutputCurrent actual value of output current
  * @param  hFantim: pointer to Fan TIM handler
  * @retval None
  * 
  * set FAN speed depending on load current 
  *
  */
void CTR_FanSpeedRegulationLoad(uint16_t hOutputCurrent, TIM_HandleTypeDef * hFantim)
{
//  static uint16_t FAN_PWMDutyCycle; // local variable instead of global variable
  uint16_t FAN_PWMDutyCycle_tmp;  
  
  /* the duty cycle of fan depends only on load value */
  if(hOutputCurrent > FAN_HIGH_LOAD_THRESHOLD){
    FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_HIGH_SPEED;
  }
  else if(hOutputCurrent < FAN_LOW_LOAD_THRESHOLD){
    FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_LOW_SPEED;
  }
  else{ // linear relationship with load
    FAN_PWMDutyCycle_tmp = FAN_PWM_DUTY_LOW_SPEED + (int32_t)((FAN_ALPHA_DUTY_LOAD*(hOutputCurrent - FAN_LOW_LOAD_THRESHOLD))/1024);  
  }
  
  /* increase/decrease actual duty cycle */
  if (FAN_PWMDutyCycle < (FAN_PWMDutyCycle_tmp - FAN_PWM_DELTA_DUTY)){
    FAN_PWMDutyCycle += FAN_PWM_DELTA_DUTY;
  }
  else if(FAN_PWMDutyCycle > (FAN_PWMDutyCycle_tmp + FAN_PWM_DELTA_DUTY)){
    FAN_PWMDutyCycle -= FAN_PWM_DELTA_DUTY;
  }
  
  /* update fan PWM duty cycle */
  PWM_FanActuation(hFantim, FAN_PWM_TIM_CHANNEL, FAN_PWMDutyCycle);
}

/**
  * @brief  Automatic turn-on/off of SR depending on output current
  * @param  hOutputCurrent actual value of output current
  * @param  hhrtim: pointer to HRTIM handle
  * @retval None
  * 
  * SR is turned on if Iout > SR_IOUT_TURN_ON_THRESHOLD, while is turned-off if Iout < SR_IOUT_TURN_OFF_THRESHOLD (SR_IOUT_TURN_ON_THRESHOLD > SR_IOUT_TURN_OFF_THRESHOLD)
  *
  */
void CTR_SRAutomaticTurnOnOffLoad(uint16_t hOutputCurrent, HRTIM_HandleTypeDef * hhrtim)
{
#define NO_BURST_VALIDATION_PERIODS_FOR_SR 20
  static uint8_t nNoBurstPeriodsCounter = NO_BURST_VALIDATION_PERIODS_FOR_SR;    
    
  /* only in run state SR can be enabled/disabled */
#ifdef OUT_CURRENT_SENSOR_CALIBRATION
  if((hOutputCurrent > hSrIoutTurnOnThreshold)&&(bDCDC_SynchRectOutputEnabled == FALSE))
#else  
  if((hOutputCurrent > SR_IOUT_TURN_ON_THRESHOLD)&&(bDCDC_SynchRectOutputEnabled == FALSE))
#endif
  {
    /* enable SR only if burst mode is disabled */
    if(bLightLoadBurstModeEnabled == FALSE){
      /* decrease validation counter */
      if(nNoBurstPeriodsCounter > 0) nNoBurstPeriodsCounter--;
      if(nNoBurstPeriodsCounter == 0){
        /* enable gate drivers for SR PWMs */
        CTR_SRGateDriverEnable();
        /* enable SR outputs */
        CTR_PWMSynchRectOutputEnable(hhrtim);
      }
    }
    else{
      /* reset validation counter */
      nNoBurstPeriodsCounter = NO_BURST_VALIDATION_PERIODS_FOR_SR;
    }
  }
#ifdef OUT_CURRENT_SENSOR_CALIBRATION
  else if ((hOutputCurrent < hSrIoutTurnOffThreshold)&&(bDCDC_SynchRectOutputEnabled == TRUE))
#else
  else if ((hOutputCurrent < SR_IOUT_TURN_OFF_THRESHOLD)&&(bDCDC_SynchRectOutputEnabled == TRUE))
#endif
  {
    /* reset validation counter */
    nNoBurstPeriodsCounter = NO_BURST_VALIDATION_PERIODS_FOR_SR;
    /* disable SR outputs */
    CTR_PWMSynchRectOutputDisable(hhrtim);
    /* disable gate drivers for SR PWMs */
//    CTR_SRGateDriverDisable();
  }
}
    
/**
  * @}
  */ 

/**
  * @}
  */ 

/******************** (C) COPYRIGHT 2017 STMicroelectronics *******************/
