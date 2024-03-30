/**
  ******************************************************************************
  * @file    LLC_Init_Periph.c
  * @author  Systems Lab 
  * @version V1.21.0
  * @date    31-May-2018
  * @brief   This file provides initializzation of all MCU peripherals
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
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "HRTIM_pwm_config_param.h"
#include "LLC_board_config_param.h"
#include "LLC_control_param.h"

#include "LLC_Init_Periph.h"
#include "LLC_Globals.h"

/** @addtogroup DSMPS_project
  * @{
  */
    
/** @addtogroup Peripheral_configuration
  * @{
  */
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WDG_OUT_VOLTAGE_MAX                     OUT_VOLT_ADC_VALUE(56)          /**< max voltage for output overvoltage fault detection [V] if VOUT_ANALOG_WATCHDOG_ENABLED is defined */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void HRTIM_ClearHandle(HRTIM_HandleTypeDef *hhrtim);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Init_Error_Handler(void)
{
  /* blocking function */
    while(1)
    {
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Init_Error_Handler();
  }
    	
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    Init_Error_Handler();
  }
}

/**
  * @brief  Configure GPIO pins for digital outputs
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
  STATUS_LED_GPIO_CLK_ENABLE();
  FAULT_LED_GPIO_CLK_ENABLE();
  SD_OD_SR1_GPIO_CLK_ENABLE();
  SD_OD_SR2_GPIO_CLK_ENABLE();
  DEBUG_DIGITAL_OUT1_GPIO_CLK_ENABLE();
 
  /* -2- Configure IOs in output push-pull */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Pin = STATUS_LED_GPIO_PIN;
  HAL_GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = FAULT_LED_GPIO_PIN;
  HAL_GPIO_Init(FAULT_LED_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SD_OD_SR1_GPIO_PIN;
  HAL_GPIO_Init(SD_OD_SR1_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = SD_OD_SR2_GPIO_PIN;
  HAL_GPIO_Init(SD_OD_SR2_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = DEBUG_DIGITAL_OUT1_GPIO_PIN;
  HAL_GPIO_Init(DEBUG_DIGITAL_OUT1_GPIO_PORT, &GPIO_InitStruct);
   
  /* -3- Reset each GPIO Output */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FAULT_LED_GPIO_PORT, FAULT_LED_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SD_OD_SR1_GPIO_PORT, SD_OD_SR1_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SD_OD_SR2_GPIO_PORT, SD_OD_SR2_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DEBUG_DIGITAL_OUT1_GPIO_PORT, DEBUG_DIGITAL_OUT1_GPIO_PIN, GPIO_PIN_RESET);
  
  /* FAN driven with a simple GPIO FAN_PWM_DRIVING is not defined */
#ifndef FAN_PWM_DRIVING
  /* enable GPIO clock */
  FAN_PWM_GPIO_CLK_ENABLE();
  
  /* configure GPIO pin */
  GPIO_InitStruct.Pin = FAN_PWM_GPIO_PIN;
  HAL_GPIO_Init(FAN_PWM_GPIO_PORT, &GPIO_InitStruct);  
  
  /* set low GPIO output */
  HAL_GPIO_WritePin(FAN_PWM_GPIO_PORT, FAN_PWM_GPIO_PIN, GPIO_PIN_RESET);
#endif
  
}
/**
  * @brief  Configure ADC peripherals to acquire analog inputs
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
  ADC_ChannelConfTypeDef sConfig1, sConfig2;
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION
  ADC_InjectionConfTypeDef sInjConfig1, sInjConfig2;
#endif
     
  /* ADCx Initialization with DMA transfer */  
  AdcHandle1.Instance                   = ADCx;  
  AdcHandle1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1; 
  AdcHandle1.Init.Resolution            = ADC_RESOLUTION12b;
  AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle1.Init.ScanConvMode          = ENABLE; /* sequence of regular or injected conversions */ 
  AdcHandle1.Init.EOCSelection          = EOC_SEQ_CONV; 
  AdcHandle1.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle1.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle1.Init.NbrOfConversion       = 3; //2;                             
  AdcHandle1.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle1.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONVHRTIM_TRG1;  /* Conversion start trigged at HRTIM_TRIG1 */ /* Conversion start trigged at each external event */
  AdcHandle1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING; 
  AdcHandle1.Init.DMAContinuousRequests = ENABLE; 
  AdcHandle1.Init.Overrun               = OVR_DATA_OVERWRITTEN; 
    
  if (HAL_ADC_Init(&AdcHandle1) != HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }

#ifdef DSMPS_CONTROL_BOARD
  /* enable sysconfig clock if not already enabled */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* remap DMA channel used for ADC2 */
  __HAL_DMA_REMAP_CHANNEL_ENABLE(ADCy_DMA_REMAP_CH); // comment if remap is not needed - see RM pag 180
#endif
  
  /* ADCy Initialization - regular Temperature measure without DMA transfer */
  AdcHandle2.Instance                   = ADCy;  
  AdcHandle2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1; 
  AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;
  AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle2.Init.ScanConvMode          = ENABLE;                       /* sequence of regular or injected conversions */ 
  AdcHandle2.Init.EOCSelection          = EOC_SEQ_CONV; 
  AdcHandle2.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle2.Init.ContinuousConvMode    = DISABLE;                      /* Continuous mode disabled to have only 1 conversion at each conversion trig */
//  AdcHandle2.Init.NbrOfConversion       = 3;
  AdcHandle2.Init.NbrOfConversion       = 1; // only Temperature is sensed
  AdcHandle2.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle2.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONVHRTIM_TRG1;  /* Conversion start trigged at HRTIM_TRIG1 */ 
  AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING; 
  AdcHandle2.Init.DMAContinuousRequests = ENABLE; 
  AdcHandle2.Init.Overrun               = OVR_DATA_OVERWRITTEN; 
  
  if (HAL_ADC_Init(&AdcHandle2) != HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }

  /* Configure ADCx regular channel - Vout sensing */
  sConfig1.Channel      = ADCx_VOUT_CHANNEL;
  sConfig1.Rank         = ADC_REGULAR_RANK_1;
  sConfig1.SamplingTime = ADCx_VOUT_CHANNEL_SAMPLING_TIME; 
  sConfig1.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig1.OffsetNumber = ADC_OFFSET_NONE;
  sConfig1.Offset = 0;

  if (HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig1) != HAL_OK)
  {
    /* Channel Configuration Error */
    Init_Error_Handler();
  }
  
#ifdef VOUT_ANALOG_WATCHDOG_ENABLED
  
  ADC_AnalogWDGConfTypeDef sAnalogWDGConf;

  sAnalogWDGConf.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  sAnalogWDGConf.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REGINJEC; 
  sAnalogWDGConf.Channel = ADCx_VOUT_CHANNEL;
  sAnalogWDGConf.ITMode = ENABLE;
  sAnalogWDGConf.HighThreshold = WDG_OUT_VOLTAGE_MAX;
  sAnalogWDGConf.LowThreshold = 0x000;

  if(HAL_ADC_AnalogWDGConfig(&AdcHandle1, &sAnalogWDGConf)!= HAL_OK)
  {
      /* AWD Configuration Error */
      Init_Error_Handler();
  }  
#endif
  
  /************************************************/
  
  /* Configure ADCx regular channel - Vbus sensing */
  sConfig1.Channel      = ADCx_VBUS_CHANNEL;
  sConfig1.Rank         = ADC_REGULAR_RANK_2;
  sConfig1.SamplingTime = ADCx_VBUS_CHANNEL_SAMPLING_TIME; 
  sConfig1.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig1.OffsetNumber = ADC_OFFSET_NONE;
  sConfig1.Offset = 0;

  if (HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig1) != HAL_OK)
  {
    /* Channel Configuration Error */
    Init_Error_Handler();
  }
  /************************************************/
    
  /* Configure ADCx regular channel - Iout sensing */
  sConfig1.Channel      = ADCx_IOUT_CHANNEL;
  sConfig1.Rank         = ADC_REGULAR_RANK_3;
  sConfig1.SamplingTime = ADCx_IOUT_CHANNEL_SAMPLING_TIME;
  sConfig1.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig1.OffsetNumber = ADC_OFFSET_NONE;
  sConfig1.Offset = 0;

  if (HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig1) != HAL_OK)
  {
    /* Channel Configuration Error */
    Init_Error_Handler();
  }
  /************************************************/
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION
   
  /* Configure ADCx injected channel - Vds_SR1 */
  sInjConfig1.InjectedSamplingTime         = ADCx_VDS1_CHANNEL_SAMPLING_TIME; 
  sInjConfig1.InjectedSingleDiff           = ADC_SINGLE_ENDED;
  sInjConfig1.InjectedOffsetNumber         = ADC_OFFSET_NONE;
  sInjConfig1.InjectedOffset               = 0;
  sInjConfig1.InjectedNbrOfConversion      = 1;
  sInjConfig1.InjectedDiscontinuousConvMode= DISABLE; 
  sInjConfig1.AutoInjectedConv             = DISABLE;
  sInjConfig1.QueueInjectedContext         = DISABLE;
  sInjConfig1.ExternalTrigInjecConv        = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG2;
  sInjConfig1.ExternalTrigInjecConvEdge    = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;  
  sInjConfig1.InjectedChannel              = ADCx_VDS1_CHANNEL;
  sInjConfig1.InjectedRank                 = ADC_INJECTED_RANK_1;
  if (HAL_ADCEx_InjectedConfigChannel(&AdcHandle1, &sInjConfig1) != HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }
  
   /* Configure ADCy injected channel - Vds_SR2 */
  sInjConfig2.InjectedSamplingTime         = ADCy_VDS2_CHANNEL_SAMPLING_TIME; 
  sInjConfig2.InjectedSingleDiff           = ADC_SINGLE_ENDED;
  sInjConfig2.InjectedOffsetNumber         = ADC_OFFSET_NONE;
  sInjConfig2.InjectedOffset               = 0;
  sInjConfig2.InjectedNbrOfConversion      = 1;
  sInjConfig2.InjectedDiscontinuousConvMode= DISABLE; 
  sInjConfig2.AutoInjectedConv             = DISABLE;
  sInjConfig2.QueueInjectedContext         = DISABLE;
  sInjConfig2.ExternalTrigInjecConv        = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG4;
  sInjConfig2.ExternalTrigInjecConvEdge    = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;  
  sInjConfig2.InjectedChannel              = ADCy_VDS2_CHANNEL;
  sInjConfig2.InjectedRank                 = ADC_INJECTED_RANK_1;
  if (HAL_ADCEx_InjectedConfigChannel(&AdcHandle2, &sInjConfig2) != HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }
#endif
  /************************************************/
  
//  /* Configure ADCy regular channel - 2nd Vout acquisition */
//  sConfig2.Channel      = ADCy_VOUT_CHANNEL;
//  sConfig2.Rank         = ADC_REGULAR_RANK_1;
//  sConfig2.SamplingTime = ADCy_VOUT_CHANNEL_SAMPLING_TIME;
//  sConfig2.SingleDiff   = ADC_SINGLE_ENDED;
//  sConfig2.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig2.Offset = 0;
//  
//  if (HAL_ADC_ConfigChannel(&AdcHandle2, &sConfig2) != HAL_OK)
//  {
//    /* Channel Configuration Error */
//    Init_Error_Handler();
//  }
  /************************************************/
  
  /* Configure ADCy regular channel - Iresonance */
//  sConfig2.Channel      = ADCy_IRES_CHANNEL;
  sConfig2.Channel      = ADCy_TEMP_CHANNEL;
  sConfig2.Rank         = ADC_REGULAR_RANK_1;
//  sConfig2.SamplingTime = ADCy_IRES_CHANNEL_SAMPLING_TIME;
  sConfig2.SamplingTime = ADCy_TEMP_CHANNEL_SAMPLING_TIME;
  sConfig2.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig2.OffsetNumber = ADC_OFFSET_NONE;
  sConfig2.Offset = 0;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle2, &sConfig2) != HAL_OK)
  {
    /* Channel Configuration Error */
    Init_Error_Handler();
  }
  /************************************************/
  
//  /* Configure ADCy regular channel - IresonanceAvg */
//  sConfig2.Channel      = ADCy_IRES_AVG_CHANNEL;
//  sConfig2.Rank         = ADC_REGULAR_RANK_2;
//  sConfig2.SamplingTime = ADCy_IRES_AVG_CHANNEL_SAMPLING_TIME;
//  sConfig2.SingleDiff   = ADC_SINGLE_ENDED;
//  sConfig2.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig2.Offset = 0;
//  
//  if (HAL_ADC_ConfigChannel(&AdcHandle2, &sConfig2) != HAL_OK)
//  {
//    /* Channel Configuration Error */
//    Init_Error_Handler();
//  }
  /************************************************/
  
//  /* Configure ADCy regular channel - Temperature */
//  sConfig2.Channel      = ADCy_TEMP_CHANNEL;
//  sConfig2.Rank         = ADC_REGULAR_RANK_3;
//  sConfig2.SamplingTime = ADCy_TEMP_CHANNEL_SAMPLING_TIME;
//  sConfig2.SingleDiff   = ADC_SINGLE_ENDED;
//  sConfig2.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig2.Offset = 0;
//  
//  if (HAL_ADC_ConfigChannel(&AdcHandle2, &sConfig2) != HAL_OK)
//  {
//    /* Channel Configuration Error */
//    Init_Error_Handler();
//  }
  /************************************************/
    
  /* enable EOS IrqHandler */
//  __HAL_ADC_ENABLE_IT(&AdcHandle1, (ADC_IT_EOS));
  
  /* start ADCx with DMA request */
  if (HAL_ADC_Start_DMA(&AdcHandle1, (uint32_t*)&DCDC_MeasureStruct.hVout, 3)!= HAL_OK) 
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }
  
#ifdef DSMPS_CONTROL_BOARD
  /* start ADCy with DMA request */
//  if (HAL_ADC_Start_DMA(&AdcHandle2, (uint32_t*)&DCDC_MeasureStruct.hIres, 3)!= HAL_OK)
  if (HAL_ADC_Start_DMA(&AdcHandle2, (uint32_t*)&DCDC_MeasureStruct.hTemperature, 1)!= HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }
#endif
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION  
  /* Start convertion of ADCx injected channel */
  if (HAL_ADCEx_InjectedStart(&AdcHandle1) != HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }
  
  /* Start convertion of ADCy injected channel */
  if (HAL_ADCEx_InjectedStart(&AdcHandle2) != HAL_OK)
  {
    /* ADC initialization Error */
    Init_Error_Handler();
  }  
#endif
    
  /* Enable the transfer complete interrupt */
//  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);

  /* Disable the Half transfer complete interrupt */
//  __HAL_DMA_DISABLE_IT((AdcHandle1.DMA_Handle), DMA_IT_HT);  
////
////  /* Disable the transfer Error interrupt */
//  __HAL_DMA_DISABLE_IT(AdcHandle1.DMA_Handle, DMA_IT_TE);
  
//  /* start ADCy with DMA request */
//  if (HAL_ADC_Start_DMA(&AdcHandle2, (uint32_t*)&DCDC_MeasureStruct.hIres, 3) != HAL_OK)
//  {
//    /* ADC initialization Error */
//    Init_Error_Handler();
//  }
  
//  /* start ADCy without DMA request */
//  if (HAL_ADC_Start(&AdcHandle2) != HAL_OK)
//  {
//    /* ADC initialization Error */
//    Init_Error_Handler();
//  }
    
}

/**
* @brief  HRTIM_ClearHandle
* @param  None
* @retval None
*/
static void HRTIM_ClearHandle(HRTIM_HandleTypeDef *hhrtim)
{
  hhrtim->Instance = (HRTIM_TypeDef *)NULL;
  
  hhrtim->hdmaMaster = (DMA_HandleTypeDef *)NULL;    
  hhrtim->hdmaTimerA = (DMA_HandleTypeDef *)NULL;     
  hhrtim->hdmaTimerB = (DMA_HandleTypeDef *)NULL;  
  hhrtim->hdmaTimerC = (DMA_HandleTypeDef *)NULL;  
  hhrtim->hdmaTimerD = (DMA_HandleTypeDef *)NULL;  
  hhrtim->hdmaTimerE = (DMA_HandleTypeDef *)NULL;  
}

/**
* @brief  HRTIM configuration
* @param  None
* @retval None
*/
void HRTIM_Config(void)
{
    HRTIM_TimeBaseCfgTypeDef timebase_config;
    HRTIM_TimerCfgTypeDef timer_config;
    HRTIM_DeadTimeCfgTypeDef deadtime_config;
    HRTIM_OutputCfgTypeDef output_config;
    HRTIM_CompareCfgTypeDef compare_config;
    HRTIM_ADCTriggerCfgTypeDef adc_trigger_config;
    HRTIM_FaultCfgTypeDef fault_config;
    HRTIM_BurstModeCfgTypeDef burst_mode_config;
    HRTIM_TimerEventFilteringCfgTypeDef TimerEventFiltering_config;
    HRTIM_EventCfgTypeDef pEventCfg;    
//    HRTIM_CaptureCfgTypeDef pCaptureCfg;
    
#ifdef OVERCURRENT_PROTECTION           
    hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_FLT_OC; 
//    hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_MASTER;
    hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
//    hhrtim.Init.SyncInputSource = HRTIM_SYNCINPUTSOURCE_INTERNALEVENT; //HRTIM_SYNCINPUTSOURCE_NONE;
//    hhrtim.Init.SyncOutputSource = HRTIM_SYNCOUTPUTSOURCE_TIMA_START; //HRTIM_SYNCOUTPUTSOURCE_MASTER_START; 
    hhrtim.Init.SyncInputSource = HRTIM_SYNCINPUTSOURCE_NONE;
    hhrtim.Init.SyncOutputSource = HRTIM_SYNCOUTPUTSOURCE_MASTER_START; 
    hhrtim.Init.SyncOutputPolarity = HRTIM_SYNCOUTPUTPOLARITY_POSITIVE; 
#endif
    
  /* ----------------------------*/
  /* HRTIM Global initialization */
  /* ----------------------------*/
  /* Use the PLLx2 clock for HRTIM */
  __HAL_RCC_HRTIM1_CONFIG(RCC_HRTIM1CLK_PLLCLK);
  
  /* Enable HRTIM clock*/
  __HRTIM1_CLK_ENABLE();

  /* Initialize HRTIM */
  HRTIM_ClearHandle(&hhrtim);
  hhrtim.Instance = HRTIM1;
  HAL_HRTIM_Init(&hhrtim);

  /* HRTIM DLL calibration: periodic calibration, set period to 14µs */
  HAL_HRTIM_DLLCalibrationStart(&hhrtim, HRTIM_CALIBRATIONRATE_14);
  /* Wait calibration completion*/
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim, DLL_CALIBRATIONTIMEOUT) != HAL_OK)
  {
    Init_Error_Handler();
  }
  
  /* --------------------------------------------------- */
  /* TIMERA and TIMERB used for Full Bridge LLC PWM generation */
  /* --------------------------------------------------- */
  timebase_config.Period = HRTIM_INIT_PWM_PERIOD; //HRTIM_MAX_PWM_PERIOD; // period set at maximum value: each timer is reset at MASTER update
  timebase_config.RepetitionCounter = HRTIM_REP_RATE;              
  timebase_config.PrescalerRatio = HRTIM_PRESC_RATIO; 
  timebase_config.Mode = HRTIM_MODE_CONTINUOUS;
    
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &timebase_config);
  
  timebase_config.Period = HRTIM_MAX_PWM_PERIOD; // period set at maximum value: each timer is reset at MASTER update
  HAL_HRTIM_TimeBaseConfig(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, &timebase_config);
  HAL_HRTIM_TimeBaseConfig(&hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, &timebase_config);
  
#ifdef SYNCH_RECTIFICATION
  HAL_HRTIM_TimeBaseConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, &timebase_config);
  HAL_HRTIM_TimeBaseConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, &timebase_config);
#endif
  
  /* FB LLC output and registers update configuration */
  timer_config.DMARequests = HRTIM_TIM_DMA_NONE;
  timer_config.DMASrcAddress = 0x0;
  timer_config.DMADstAddress = 0x0;
  timer_config.DMASize = 0x0;
  timer_config.HalfModeEnable = HRTIM_HALFMODE_ENABLED; 
  timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED; //HRTIM_SYNCSTART_ENABLED; 
  timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED; //HRTIM_SYNCRESET_ENABLED; //
  timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;  
  timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;  
  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED; // HRTIM_TIMUPDATEONRESET_DISABLED; //MODIFIED
  timer_config.InterruptRequests = HRTIM_TIM_IT_NONE; 
  timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT_OC; 
  timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;  // complementary outputs with deadtime
  timer_config.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED; //HRTIM_TIMDELAYEDPROTECTION_DELAYEDBOTH_EEV68; //HRTIM_TIMDELAYEDPROTECTION_DISABLED; ////HRTIM_TIMDELAYEDPROTECTION_ENABLED; // VERIFICARE DELAYED IDLE!!!
//  timer_config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
//  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  
  timer_config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_A; //HRTIM_TIMUPDATETRIGGER_MASTER; // solution without MASTER_TIM
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE; //HRTIM_TIMRESETTRIGGER_MASTER_PER; //solution without MASTER_TIM
  
  /* MASTER_TIMER config */
  timer_config.InterruptRequests = HRTIM_TIM_IT_NONE; //HRTIM_TIM_IT_UPD; 
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER,&timer_config);
  
  /* PWM_FB_HS1_LS1_HRTIM_TIMER config */
//  timer_config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_A;
//  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_UPDATE; 
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, &timer_config);
  
  /* PWM_FB_HS2_LS2_HRTIM_TIMER config */
//  timer_config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_B;
//  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_UPDATE; 
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, &timer_config);
  
  
  
  /* ---------------- SET TIMC and TIMD CONFIG for SR ---------------- */
  
#ifdef SYNCH_RECTIFICATION
  timer_config.DMARequests = HRTIM_TIM_DMA_NONE;
  timer_config.DMASrcAddress = 0x0;
  timer_config.DMADstAddress = 0x0;
  timer_config.DMASize = 0x0;
  timer_config.HalfModeEnable = HRTIM_HALFMODE_DISABLED; 
  timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED; //HRTIM_SYNCSTART_ENABLED; 
  timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED; //HRTIM_SYNCRESET_ENABLED;
//  timer_config.StartOnSync = HRTIM_SYNCSTART_ENABLED; 
//  timer_config.ResetOnSync = HRTIM_SYNCRESET_ENABLED;
  timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  
  timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
//  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED; 
  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED; 
  timer_config.InterruptRequests = HRTIM_TIM_IT_NONE; 
  timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT_OC; 
  timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;  
  timer_config.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED; //HRTIM_TIMDELAYEDPROTECTION_DELAYEDBOTH_EEV68; //HRTIM_TIMDELAYEDPROTECTION_ENABLED; // VERIFICARE DELAYED IDLE!!!
//  timer_config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
//  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  
  timer_config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_A;   // solution without master
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_UPDATE; //HRTIM_TIMRESETTRIGGER_NONE; // // solution without master
  
  /* SR_HS2_LS1 output and registers update configuration */
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, &timer_config);
  
  /* SR_HS1_LS2 output and registers update configuration */
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, &timer_config);
#endif
  
  /* ---------------------------*/
  /* HRTIM Output configuration */
  /* ---------------------------*/
  
  /* Config - PWM_FB_HS1 */
  output_config.Polarity = HRTIM_PWM_FB_HIGHSIDE_OUTPOLARITY;
//  output_config.SetSource = HRTIM_OUTPUTSET_MASTERPER; 
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_MASTERCMP1; // + HRTIM_OUTPUTRESET_AWDG1_EEV; // TO VERIFY AT SHUT-DOWN (SEEMS OK)
  output_config.SetSource = HRTIM_OUTPUTSET_TIMPER;  // solution without MASTER TIM
  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1; // + HRTIM_OUTPUTRESET_AWDG1_EEV; // TO VERIFY AT SHUT-DOWN (SEEMS OK) - solution without MASTER TIM
  output_config.IdleMode = HRTIM_OUTPUTIDLEMODE_IDLE; 
  output_config.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  output_config.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_DELAYED; //HRTIM_OUTPUTBURSTMODEENTRY_REGULAR; // CHANGED: to verify!!!
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, PWM_FB_HS1_HRTIM_OUTPUT, &output_config);

  /* Config - PWM_FB_LS1 */
  output_config.Polarity = HRTIM_PWM_FB_LOWSIDE_OUTPOLARITY;
  output_config.SetSource = HRTIM_OUTPUTSET_NONE;       // complementary mode used
  output_config.ResetSource  = HRTIM_OUTPUTRESET_NONE;  // complementary mode used
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, PWM_FB_LS1_HRTIM_OUTPUT, &output_config);
  
  /* Config - PWM_FB_HS2 */
  output_config.Polarity = HRTIM_PWM_FB_HIGHSIDE_OUTPOLARITY;
//  output_config.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;  
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_MASTERPER; // + HRTIM_OUTPUTRESET_AWDG1_EEV; // TO VERIFY AT SHUT-DOWN (SEEMS OK)
  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP1;    // solution without MASTER TIM
  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMPER; // + HRTIM_OUTPUTRESET_AWDG1_EEV; // TO VERIFY AT SHUT-DOWN (SEEMS OK)
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, PWM_FB_HS2_HRTIM_OUTPUT, &output_config);

  /* Config - PWM_FB_LS2 */
  output_config.Polarity = HRTIM_PWM_FB_LOWSIDE_OUTPOLARITY;
  output_config.SetSource = HRTIM_OUTPUTSET_NONE;      // complementary mode used
  output_config.ResetSource  = HRTIM_OUTPUTRESET_NONE; // complementary mode used
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, PWM_FB_LS2_HRTIM_OUTPUT, &output_config);
  
#ifdef SYNCH_RECTIFICATION  
  /* Common Config */
//  output_config.BurstModeEntryDelayed =  HRTIM_OUTPUTBURSTMODEENTRY_DELAYED; //HRTIM_OUTPUTBURSTMODEENTRY_REGULAR; //changed  TO TEST!!! //
  
  /* Config - PWM_SR_HS2 */
  output_config.Polarity = HRTIM_PWM_SR_HIGHSIDE_OUTPOLARITY;  
  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP2;
  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR1_COMP_EEV; // V1.5: Second condition can be verified only for Adaptive SR
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR1_COMP_EEV + HRTIM_OUTPUTRESET_MASTERCMP1; // Second condition can be verified only for Adaptive SR, 3rd condition limits at zero minimum falling delay of SR and avoid resynch issues
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_MASTERCMP1; // TEST FOR FAULT
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 /*+ HRTIM_OUTPUTRESET_TIMEV_2 */; // TEST FOR FAULT
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, PWM_SR_HS2_HRTIM_OUTPUT, &output_config);
  
  /* Config - PWM_SR_HS2 - DEBUG ADCy injected trigger point with PWM waveform */
//  output_config.Polarity = HRTIM_PWM_SR_HIGHSIDE_OUTPOLARITY;  
//  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP4;
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP4 + 1000;
//  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, PWM_SR_HS2_HRTIM_OUTPUT, &output_config);
  
  /* Config - PWM_SR_LS1 */
  output_config.Polarity = HRTIM_PWM_SR_LOWSIDE_OUTPOLARITY;
  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP2; 
  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR1_COMP_EEV; // Second condition can be verified only for Adaptive SR
//   output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR1_COMP_EEV + HRTIM_OUTPUTRESET_MASTERCMP1; // V1.5: Second condition can be verified only for Adaptive SR, 3rd condition limits at zero minimum falling delay of SR and avoid resynch issues
//   output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_MASTERCMP1; // TEST FOR FAULT
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 /*+ HRTIM_OUTPUTRESET_TIMEV_2 */; // TEST FOR FAULT
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, PWM_SR_LS1_HRTIM_OUTPUT, &output_config);
  
  /* Config - PWM_SR_HS1 */
  output_config.Polarity = HRTIM_PWM_SR_HIGHSIDE_OUTPOLARITY;
  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP2; 
  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR2_COMP_EEV;  // // V1.5: Second condition can be verified only for Adaptive SR
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_MASTERPER; // TEST FOR FAULT
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 /*+ HRTIM_OUTPUTRESET_TIMPER*/; // TEST FOR FAULT
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR2_COMP_EEV + HRTIM_OUTPUTRESET_MASTERPER; // 2nd condition can be verified only for Adaptive SR, 3rd condition limits at zero minimum falling delay of SR and avoid resynch issues
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, PWM_SR_HS1_HRTIM_OUTPUT, &output_config);
  
  /* Config PWM_SR_HS1 - DEBUG ADCy injected trigger point with PWM waveform */
//  output_config.Polarity = HRTIM_PWM_SR_HIGHSIDE_OUTPOLARITY;
//  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP4; 
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP4 + 10000; 
//  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, PWM_SR_HS1_HRTIM_OUTPUT, &output_config);
  
  /* Config - PWM_SR_LS2 */
  output_config.Polarity = HRTIM_PWM_SR_LOWSIDE_OUTPOLARITY;
  output_config.SetSource = HRTIM_OUTPUTSET_TIMCMP2; 
  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR2_COMP_EEV; // Second condition can be verified only for Adaptive SR 
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_SR2_COMP_EEV + HRTIM_OUTPUTRESET_MASTERPER; // V1.5: 2nd condition can be verified only for Adaptive SR, 3rd condition limits at zero minimum falling delay of SR and avoid resynch issues
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 + HRTIM_OUTPUTRESET_MASTERPER; // TEST FOR FAULT
//  output_config.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1 /*+ HRTIM_OUTPUTRESET_TIMPER*/; // TEST FOR FAULT
  HAL_HRTIM_WaveformOutputConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, PWM_SR_LS2_HRTIM_OUTPUT, &output_config);
  
  /* Set compare 1 and 2 registers for SR outputs */
  
  /* PWM_SR_HS2/PWM_SR_LS1 set */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR; 
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = SYNCH_RECT_DELAY_RISING1_INIT_HR_TICKS + DEAD_TIME_RISING_HR_TICKS; 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, &compare_config);
  
  /* PWM_SR_HS2/PWM_SR_LS1 reset */
  compare_config.CompareValue = (uint16_t)((uint16_t)(HRTIM_INIT_PWM_PERIOD/2) - SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS); 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, &compare_config);
  
  /* PWM_SR_HS1/PWM_SR_LS2 set */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (uint16_t)(HRTIM_INIT_PWM_PERIOD/2) + SYNCH_RECT_DELAY_RISING2_INIT_HR_TICKS + DEAD_TIME_FALLING_HR_TICKS; 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, &compare_config);
  
  /* PWM_SR_HS1/PWM_SR_LS2 reset */
  compare_config.CompareValue = (uint16_t)((HRTIM_INIT_PWM_PERIOD) - SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS); 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, &compare_config);
#endif  
      
#ifdef ADAPTIVE_SYNCH_RECTIFICATION   
  /* Set COMP3 for Turn-off blanking window (Ton_min) for SR1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR; 
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (uint16_t)(SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS + SYNCH_RECT_DELAY_RISING1_INIT_HR_TICKS + DEAD_TIME_RISING_HR_TICKS); 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, &compare_config); 
  
  /* Set COMP3 for Turn-off blanking window (Ton_min) for SR2 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (uint16_t)(SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS + (uint16_t)(HRTIM_INIT_PWM_PERIOD/2) + SYNCH_RECT_DELAY_RISING2_INIT_HR_TICKS + DEAD_TIME_FALLING_HR_TICKS); 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, &compare_config); 
  
      
  /* ------------------------------ */
  /* COMP filters programming */
  /* ------------------------------ */
  /* turnoff (EEV signal) blanking done with CMP3 FOR SR1 */
  TimerEventFiltering_config.Filter = HRTIM_TIMEVENTFILTER_BLANKINGCMP3; 
  TimerEventFiltering_config.Latch = HRTIM_TIMEVENTLATCH_DISABLED;
  HAL_HRTIM_TimerEventFilteringConfig(&hhrtim,
                                  PWM_SR_HS2_LS1_HRTIM_TIMERINDEX,
                                  HRTIM_SR1_COMP_EVENT,
                                  &TimerEventFiltering_config);
  
  /* turnoff (EEV signal) blanking done with CMP3 FOR SR2 */
  TimerEventFiltering_config.Filter = HRTIM_TIMEVENTFILTER_BLANKINGCMP3; 
  TimerEventFiltering_config.Latch = HRTIM_TIMEVENTLATCH_DISABLED;
  HAL_HRTIM_TimerEventFilteringConfig(&hhrtim,
                                  PWM_SR_HS1_LS2_HRTIM_TIMERINDEX,
                                  HRTIM_SR2_COMP_EVENT,
                                  &TimerEventFiltering_config);
  
  /* ------------------------------------------------- */
  /* Capture units programming for autodelayed compare */
  /* ------------------------------------------------- */
  /* SR1_COMP_EVENT config */
  HAL_HRTIM_EventPrescalerConfig(&hhrtim, HRTIM_EVENTPRESCALER_DIV1);
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_HIGH;
  pEventCfg.Sensitivity = HRTIM_EVENTSENSITIVITY_LEVEL;
  pEventCfg.Filter = HRTIM_EVENTFILTER_3;
  pEventCfg.FastMode = HRTIM_EVENTFASTMODE_DISABLE;  
  pEventCfg.Source = HRTIM_SR1_COMP_EVENTSRC; 
  HAL_HRTIM_EventConfig(&hhrtim, HRTIM_SR1_COMP_EVENT, &pEventCfg);
  
  /* SR2_COMP_EVENT config */
  pEventCfg.Source = HRTIM_SR2_COMP_EVENTSRC; 
  HAL_HRTIM_EventConfig(&hhrtim, HRTIM_SR2_COMP_EVENT, &pEventCfg);
      
  /* SR1 Capture trigger config */
//  pCaptureCfg.Trigger = HRTIM_CAPTURETRIGGER_SR1_COMP_EEV; 
//  HAL_HRTIM_WaveformCaptureConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_CAPTUREUNIT_1, &pCaptureCfg);
  
  /* SR2 Capture trigger config */
//  pCaptureCfg.Trigger = HRTIM_CAPTURETRIGGER_SR2_COMP_EEV; 
//  HAL_HRTIM_WaveformCaptureConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_CAPTUREUNIT_2, &pCaptureCfg);
  
#endif
   
  /* -----------------------------------------*/
  /* Dead time insertion - only PWM_HS/LS 1&2 */
  /* -----------------------------------------*/
  
  deadtime_config.Prescaler = DEAD_TIME_PRESCALER; 
  
  deadtime_config.RisingValue = DEAD_TIME_RISING_DT_TICKS;
  deadtime_config.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  deadtime_config.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  deadtime_config.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  
  deadtime_config.FallingValue = DEAD_TIME_FALLING_DT_TICKS;
  deadtime_config.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE; 
  deadtime_config.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  deadtime_config.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;  
  
  HAL_HRTIM_DeadTimeConfig(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, &deadtime_config);
  
  /* exchanged dead times for second leg */
  deadtime_config.RisingValue = DEAD_TIME_FALLING_DT_TICKS;
  deadtime_config.FallingValue = DEAD_TIME_RISING_DT_TICKS;
  HAL_HRTIM_DeadTimeConfig(&hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, &deadtime_config);

  /* --------------------------*/
  /* ADC trigger intialization */
  /* --------------------------*/
  
  /* ADC trigger1 on Master_CMP2 */
//  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;  //V1.5
//  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP2; //V1.5
  /* ADC trigger3 on TIMA_CMP2 */
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_1, &adc_trigger_config) != HAL_OK)
  {
    /* ADC Trigger Error */
    Init_Error_Handler();
  }
  
  /* ADC trigger3 on Master_CMP3 */
//  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;  //V1.5
//  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP3; //V1.5
  /* ADC trigger3 on TIMA_CMP3 */
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_3, &adc_trigger_config) != HAL_OK)
  {
    /* ADC Trigger Error */
    Init_Error_Handler();
  }
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION
  /* ADC trigger and compare register for SR1 */
  /* ADC trigger2 on SR1_TIM CMP4 */
//  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER; //V1.5
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A; 
  adc_trigger_config.Trigger = HRTIM_INJECT_ADCTRIGGEREVENT_SR1;     
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_2, &adc_trigger_config) != HAL_OK)
  {
    /* ADC Trigger Error */
    Init_Error_Handler();
  }
  
  /* Set SR1_TIM compare 4 register for ADCx injected trigger point */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1; //HRTIM_AUTODELAYEDMODE_REGULAR; 
  compare_config.AutoDelayedTimeout = (uint16_t)((uint16_t)(HRTIM_INIT_PWM_PERIOD/2) - SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS);
  compare_config.CompareValue = ADCx_INJECTED_TRIGGER_POINT_HR_TICKS; 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_4, &compare_config);
  /***************************************************************************/
  
  /* ADC trigger and compare register for SR2 */
  /* ADC trigger2 on SR2_TIM CMP4 */
//  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER; //V1.5
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A; 
  adc_trigger_config.Trigger = HRTIM_INJECT_ADCTRIGGEREVENT_SR2;     
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_4, &adc_trigger_config) != HAL_OK)
  {
    /* ADC Trigger Error */
    Init_Error_Handler();
  }
  
#ifdef AUTODELAYED_ADCy_INJECTED_TRIGGER
  /* Set SR2_TIM compare 4 register for ADCy injected trigger point in autodelayed mode */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1; 
  compare_config.AutoDelayedTimeout = (uint16_t)((HRTIM_INIT_PWM_PERIOD) - SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS);
  compare_config.CompareValue = ADCy_INJECTED_TRIGGER_POINT_HR_TICKS; 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_4, &compare_config);
#else
  /* Set SR2_TIM compare 4 register for ADCy injected trigger point in regular mode */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR; 
  compare_config.AutoDelayedTimeout = 0; 
  compare_config.CompareValue = (uint16_t)(HRTIM_INIT_PWM_PERIOD - SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS + ADCy_INJECTED_TRIGGER_POINT_HR_TICKS); 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_4, &compare_config);
#endif
  
#endif  
  
  /* Set MASTER_CMP2 for ADCx regular sequence's trigger */
//  __HAL_HRTIM_SetCompare(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, ADCx_REGULAR_TRIGGER_POINT_HR_TICKS);
  
  /* Set TIMA_CMP2 for ADCx regular sequence's trigger */
  __HAL_HRTIM_SetCompare(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, ADCx_REGULAR_TRIGGER_POINT_HR_TICKS);
  
  /* Set MASTER_CMP2 for ADCx regular sequence's trigger - DEBUG */
//  __HAL_HRTIM_SetCompare(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, (HRTIM_INIT_PWM_PERIOD/2 + ADCy_TRIGGER_POINT_HR_TICKS)); //(HRTIM_INIT_PWM_PERIOD/4));
  
  /* Set MASTER_CMP3 for ADCy regular sequence's trigger */
//  __HAL_HRTIM_SetCompare(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, (HRTIM_INIT_PWM_PERIOD/2 + ADCy_REGULAR_TRIGGER_POINT_HR_TICKS));
  
  /* Set TIMA_CMP3 for ADCy regular sequence's trigger */
  __HAL_HRTIM_SetCompare(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (HRTIM_INIT_PWM_PERIOD/2 + ADCy_REGULAR_TRIGGER_POINT_HR_TICKS));
  
  /* Set MASTER_CMP3 for ADCy regular sequence's trigger in autodelayed mode */
//  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1; //HRTIM_AUTODELAYEDMODE_REGULAR; 
//  compare_config.AutoDelayedTimeout = HRTIM_INIT_PWM_PERIOD/2;
//  compare_config.CompareValue = ADCy_REGULAR_TRIGGER_POINT_HR_TICKS;
//  HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, &compare_config);
  
  /* ---------------------*/
  /* FAULT initialization */
  /* ---------------------*/
#ifdef OVERCURRENT_PROTECTION
  /* set prescaler for fFLTS */
  HAL_HRTIM_FaultPrescalerConfig(&hhrtim, HRTIM_FAULTPRESCALER_DIV1);
  
  /* select fSAMPLING and number of sample for filtering (see RM pag 680) */
//  fault_config.Filter = HRTIM_FAULTFILTER_3;    /* fSAMPLING = fHRTIM, N=8: with fFLTS = f_HRTIM = 144MHz -> t_filt = 55.56ns */
//  fault_config.Filter = HRTIM_FAULTFILTER_4;    /* fSAMPLING = fFLTS/2, N=6: with fFLTS = f_HRTIM = 144MHz -> t_filt = 83.33ns */
//  fault_config.Filter = HRTIM_FAULTFILTER_5;    /* fSAMPLING = fFLTS/2, N=8: with fFLTS = f_HRTIM = 144MHz -> t_filt = 111ns */
//  fault_config.Filter = HRTIM_FAULTFILTER_6;    /* fSAMPLING = fFLTS/4, N=6: with fFLTS = f_HRTIM = 144MHz ->  t_filt = 166.67ns */
//  fault_config.Filter = HRTIM_FAULTFILTER_7;    /* fSAMPLING = fFLTS/4, N=8: with fFLTS = f_HRTIM = 144MHz ->  t_filt = 222.22ns */ // FW V1.5
  fault_config.Filter = HRTIM_FAULTFILTER_8;    /* fSAMPLING = fFLTS/8, N=6: with fFLTS = f_HRTIM = 144MHz ->  t_filt = 333.33ns */ // FW V1.6
  fault_config.Lock = HRTIM_FAULTLOCK_READWRITE;
  fault_config.Polarity = HRTIM_FAULTPOLARITY_HIGH; 
  fault_config.Source = HRTIM_FAULTSOURCE_INTERNAL; 
     
  HAL_HRTIM_FaultConfig(&hhrtim, HRTIM_FAULT_OC, &fault_config);
//  HAL_HRTIM_FaultModeCtl(&hhrtim, HRTIM_FAULT_OC, HRTIM_FAULTMODECTL_ENABLED);  // comment to disable OC protection at start-up
#endif
  
  /* --------------------------*/
  /* BURST MODE initialization */
  /* --------------------------*/
#ifdef LIGHT_LOAD_BURST_MODE
  /* test for Burst mode */
  burst_mode_config.Mode = HRTIM_BURSTMODE_CONTINOUS; 
  burst_mode_config.ClockSource = HRTIM_BURSTMODECLOCKSOURCE_MASTER; 
  burst_mode_config.Prescaler = HRTIM_BURSTMODEPRESCALER_DIV1; 
  burst_mode_config.PreloadEnable = HRIM_BURSTMODEPRELOAD_ENABLED;
  burst_mode_config.Trigger = HRTIM_BURSTMODETRIGGER_MASTER_REPETITION; 
  burst_mode_config.IdleDuration = BURST_MODE_IDLE_DURATION_INIT; 
  burst_mode_config.Period = BURST_MODE_PERIOD_INIT; 
    
  if (HAL_HRTIM_BurstModeConfig(&hhrtim, &burst_mode_config) != HAL_OK)
  {
    Init_Error_Handler();
  }
  
  HAL_HRTIM_BurstModeSoftwareTrigger(&hhrtim);
#endif 
  
  /* ---------------*/
  /* HRTIM start-up */
  /* ---------------*/
  /* Enable HRTIM's outputs TD1 and TD2 */
  /* Note: it is necessary to enable also GPIOs to have outputs functional */
  
#ifdef SYNCH_RECTIFICATION
  /* Start HRTIM's TIMERS for FB LLC, SR and Master TIMER */
  HAL_HRTIM_WaveformCounterStart_IT(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERID | PWM_FB_HS2_LS2_HRTIM_TIMERID | PWM_SR_HS2_LS1_HRTIM_TIMERID | PWM_SR_HS1_LS2_HRTIM_TIMERID | HRTIM_TIMERID_MASTER);
#else
  /* Start HRTIM's TIMERS for FB LLC and Master TIMER */
  HAL_HRTIM_WaveformCounterStart_IT(&hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERID | PWM_FB_HS2_LS2_HRTIM_TIMERID | HRTIM_TIMERID_MASTER);
#endif
  
}

/**
  * @brief  Configure timer TIM7 to schedule Vout control loop
  * @param  None
  * @retval None
  */
void TIM6_VoutControl_Config(void)
{  
  /*##-1- Configure the TIM peripheral #######################################*/
  /* Configure TIM7 */
  /* PWM configuration */
  TimVoutControlHandle.Instance = TIM6;
    
  /* Time Base configuration */
  TimVoutControlHandle.Init.Prescaler = VOLTAGE_CONTROL_TIM_PRSC; 
  TimVoutControlHandle.Init.Period =((uint16_t) (SystemCoreClock / (uint32_t)(VOLTAGE_CONTROL_LOOP_FREQ_HZ * (VOLTAGE_CONTROL_TIM_PRSC + 1))) -1);
  TimVoutControlHandle.Init.ClockDivision = 0;    
  TimVoutControlHandle.Init.CounterMode = TIM_COUNTERMODE_UP; 
  TimVoutControlHandle.Init.RepetitionCounter = 0;
  
  HAL_TIM_Base_Init(&TimVoutControlHandle);
  
  /* start Timer with update event */
  HAL_TIM_Base_Start_IT(&TimVoutControlHandle);
}

/**
  * @brief  Configure FAN_PWM_TIM peripheral for generation of PWM to drive the FAN
  * @param  None
  * @retval None
  */
void FAN_TIM_Config(void)
{ 
  TIM_OC_InitTypeDef      TIMPWM_Config;
      
  /*##-1- Configure the TIM peripheral #######################################*/
  /* Configure FAN_PWM_TIM */
  /* PWM configuration */
  FanPwmHandle.Instance = FAN_PWM_TIM;
  
  /* Time Base configuration: Channel 2 and channel 5 frequency is 
     APB2 clock / period = 72000000 / 50000 = 1440 Hz */
  FanPwmHandle.Init.Prescaler = FAN_PWM_TIM_PRESCALER; 
  FanPwmHandle.Init.Period = FAN_PWM_TIM_PERIOD;
  FanPwmHandle.Init.ClockDivision = 0;    
  FanPwmHandle.Init.CounterMode = TIM_COUNTERMODE_UP; 
  FanPwmHandle.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&FanPwmHandle);
    
  /*##-2- Configure the PWM Output Capture ########################################*/  
  /* PWM Output Capture configuration of TIM 3 channel 4 */
  TIMPWM_Config.OCMode  = TIM_OCMODE_PWM1;
  TIMPWM_Config.OCIdleState = TIM_OCIDLESTATE_RESET;
  TIMPWM_Config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  TIMPWM_Config.Pulse = FAN_PWM_INIT_DUTY;
  TIMPWM_Config.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIMPWM_Config.OCNPolarity = TIM_OCNPOLARITY_LOW;
  TIMPWM_Config.OCFastMode = TIM_OCFAST_DISABLE;
  if(HAL_TIM_PWM_ConfigChannel(&FanPwmHandle, &TIMPWM_Config, FAN_PWM_TIM_CHANNEL) != HAL_OK)
  {
    /* Initialization Error */
    Init_Error_Handler();
  }
  
  /*##-3- output enable ##########################*/
//  if(HAL_TIM_PWM_Start(&FanPwmHandle, FAN_PWM_TIM_CHANNEL)!= HAL_OK)
//  {
//    Init_Error_Handler();
//  }
}

/**
  * @brief  Configure timer TIM6 to schedule low frequency task
  * @param  None
  * @retval None
  *
  * Low frequency task: temperature and Vin filtering, fan speed control, update config parameters and manage enable/disable of SR and Burst mode
  */
void TIM16_LowFrequencyTask_Config(void)
{  
  /*##-1- Configure the TIM peripheral #######################################*/
  /* Configure TIM16 */
  TimTempFanCtrlHandle.Instance = TIM16;
  
  /* Time Base configuration: Channel 2 and channel 5 frequency is 
     APB2 clock / period = 72000000 / 50000 = 1440 Hz */
  TimTempFanCtrlHandle.Init.Prescaler = LOW_FREQUENCY_TASK_TIM_PRSC; 
  TimTempFanCtrlHandle.Init.Period =((uint16_t) (SystemCoreClock / (uint32_t)(LOW_FREQUENCY_TASK_FREQ_HZ * (LOW_FREQUENCY_TASK_TIM_PRSC + 1))) -1);
  TimTempFanCtrlHandle.Init.ClockDivision = 0;    
  TimTempFanCtrlHandle.Init.CounterMode = TIM_COUNTERMODE_UP; 
  TimTempFanCtrlHandle.Init.RepetitionCounter = 0;
  
  HAL_TIM_Base_Init(&TimTempFanCtrlHandle);
  
  /* start Timer with update event */
  HAL_TIM_Base_Start_IT(&TimTempFanCtrlHandle);
}

/**
  * @brief  Configure COMP peripherals for overcurrent detection
  * @param  None
  * @retval None
  */
void COMP_OverCurrProtection_Config(void)
{   
  /*##-1- Configure the COMPx peripheral ###################################*/
  /* COMP_OC Init: the threshold is set on DAC_OC_CHANNEL */
  CompOCHandle.Instance = COMP_OC;

  CompOCHandle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC_OC; 
//  CompOCHandle.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;     // not available on STM32F334
//  CompOCHandle.Init.Hysteresis = COMP_HYSTERESIS_HIGH;                  // not available on STM32F334
//  CompOCHandle.Init.Mode = COMP_MODE_HIGHSPEED;                         // not available on STM32F334
  CompOCHandle.Init.Output = COMP_OUTPUT_NONE;
  CompOCHandle.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  CompOCHandle.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE; //COMP_BLANKINGSRCE_TIM2OC3; 
  CompOCHandle.Init.WindowMode = COMP_WINDOWMODE_DISABLED;
  CompOCHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING; // COMP_TRIGGERMODE_EVENT_RISING; 

  if(HAL_COMP_Init(&CompOCHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Init_Error_Handler();    
  }

  /*##-2- Start the comparator process #####################################*/
  
  /* Enable COMP_OC without IT: the higher threshold is set on DAC_OC_CHANNEL */
  HAL_COMP_Start(&CompOCHandle);

}

/**
  * @brief  Configure COMP peripherals for adaptive SR
  * @param  None
  * @retval None
  */
void COMP_AdaptiveSR_Config(void)
{   
  /*##-1A- Configure the COMP_VDS_SR1 peripheral ###################################*/
  /* COMP_VDS_SR1 Init: the threshold is set on DAC_SR_CHANNEL1 */
  CompSRHandle1.Instance = COMP_VDS_SR1;
  CompSRHandle1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC_VDS_SR1; 
//  CompSRHandle1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;    // not available on STM32F334
//  CompSRHandle1.Init.Hysteresis = COMP_HYSTERESIS_HIGH;                 // not available on STM32F334
//  CompSRHandle1.Init.Mode = COMP_MODE_HIGHSPEED;                        // not available on STM32F334
  CompSRHandle1.Init.Output = COMP_OUTPUT_NONE;
  CompSRHandle1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  CompSRHandle1.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;     // EEV blanked in HRTIM
  CompSRHandle1.Init.WindowMode = COMP_WINDOWMODE_DISABLED;
  CompSRHandle1.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING; 

  if(HAL_COMP_Init(&CompSRHandle1) != HAL_OK)
  {
    /* Initiliazation Error */
    Init_Error_Handler();    
  }
  
  /*##-1B- Configure the COMP_VDS_SR2 peripheral ###################################*/
  /* COMP_VDS_SR1 Init: the threshold is set on DAC_SR_CHANNEL2 */
  CompSRHandle2.Instance = COMP_VDS_SR2;

  CompSRHandle2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC_VDS_SR2; 
//  CompSRHandle2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;    // not available on STM32F334
//  CompSRHandle2.Init.Hysteresis = COMP_HYSTERESIS_HIGH;                 // not available on STM32F334
//  CompSRHandle2.Init.Mode = COMP_MODE_HIGHSPEED;                        // not available on STM32F334
  CompSRHandle2.Init.Output = COMP_OUTPUT_NONE;
  CompSRHandle2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  CompSRHandle2.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;     // EEV blanked in HRTIM
  CompSRHandle2.Init.WindowMode = COMP_WINDOWMODE_DISABLED;
  CompSRHandle2.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING; 

  if(HAL_COMP_Init(&CompSRHandle2) != HAL_OK)
  {
    /* Initiliazation Error */
    Init_Error_Handler();    
  }

  /*##-2A- Start the comparator process #####################################*/ 
  /* Enable COMP_SR1 with IT: the higher threshold is set on DAC_SR_CHANNEL1 */
//  HAL_COMP_Start_IT(&CompSRHandle1);
  
  /*##-2B- Start the comparator process #####################################*/ 
  /* Enable COMP_SR2 with IT: the higher threshold is set on DAC_SR_CHANNEL2 */
//  HAL_COMP_Start_IT(&CompSRHandle2);
  
  
  /* Enable COMP_SR1 without IT: the higher threshold is set on DAC_SR_CHANNEL1  */
//  HAL_COMP_Start(&CompSRHandle1);  
//  
//  /* Enable COMP_SR2 without IT: the higher threshold is set on DAC_SR_CHANNEL2  */
//  HAL_COMP_Start(&CompSRHandle2);

}
/**
  * @brief  Configure DAC peripheral to set OC protection threshold
  * @param  None
  * @retval None
  */
void DAC_OverCurrProtection_Config(void)
{ 
  DAC_ChannelConfTypeDef   DacOCsConfig;           /**< handle for DAC_OC Channel */
  
  /*##-1- Configure the DAC peripheral #######################################*/
  DacOCHandle.Instance = DAC_OC;
  
  if(HAL_DAC_Init(&DacOCHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Init_Error_Handler();    
  }

  /*##-2- Configure DAC channel1 #############################################*/  
  DacOCsConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  DacOCsConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  DacOCsConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
    
  if(HAL_DAC_ConfigChannel(&DacOCHandle, &DacOCsConfig, DAC_OC_CHANNEL) != HAL_OK)
    {
    /* Channel configuration Error */
    Init_Error_Handler();    
    }
    
  /*##-3- Set DAC Channel1 DHR register ######################################*/ 
  if(HAL_DAC_SetValue(&DacOCHandle, DAC_OC_CHANNEL, DAC_ALIGN_12B_R, DAC_OVERCURRENT_COMP_THRESHOLD) != HAL_OK) 
  {
    /* Setting value Error */
    Init_Error_Handler();        
  }
  
  /*##-4- Enable DAC Channel1 ################################################*/ 
  if(HAL_DAC_Start(&DacOCHandle, DAC_OC_CHANNEL) != HAL_OK)
  {
    /* Start Error */
    Init_Error_Handler();      
  }
  
}

/**
  * @brief  Configure DAC peripheral to set SR turn-on/-off thresholds
  * @param  None
  * @retval None
  */
void DAC_AdaptiveSR_Config(void)
{ 
  DAC_ChannelConfTypeDef   DacSRsConfig1;          /**< handle for DAC_SR Channel1 */
  DAC_ChannelConfTypeDef   DacSRsConfig2;          /**< handle for DAC_SR Channel2 */

  /*##-1- Configure the DAC peripheral #######################################*/
  DacSRHandle.Instance = DAC_SR;
  
  if(HAL_DAC_Init(&DacSRHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Init_Error_Handler();    
  }

  /*##-2A- Configure DAC SR channel1 #############################################*/  
  DacSRsConfig1.DAC_Trigger = DAC_TRIGGER_NONE;
  DacSRsConfig1.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE; 
  DacSRsConfig1.DAC_OutputSwitch = DAC_OUTPUTSWITCH_DISABLE;
    
  if(HAL_DAC_ConfigChannel(&DacSRHandle, &DacSRsConfig1, DAC_SR_CHANNEL1) != HAL_OK)
    {
    /* Channel configuration Error */
    Init_Error_Handler();    
    }
  
  /*##-2B- Configure DAC SR channel2 #############################################*/  
  DacSRsConfig2.DAC_Trigger = DAC_TRIGGER_NONE;
  DacSRsConfig2.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  DacSRsConfig2.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
    
  if(HAL_DAC_ConfigChannel(&DacSRHandle, &DacSRsConfig2, DAC_SR_CHANNEL2) != HAL_OK)
    {
    /* Channel configuration Error */
    Init_Error_Handler();    
    }
    
  /*##-3A- Set DAC SR Channel1 DHR register ######################################*/ 
  if(HAL_DAC_SetValue(&DacSRHandle, DAC_SR_CHANNEL1, DAC_ALIGN_12B_R, DAC_SR1_TURN_OFF_COMP_THRESHOLD) != HAL_OK) 
  {
    /* Setting value Error */
    Init_Error_Handler();        
  }
  
  /*##-3B- Set DAC SR Channel2 DHR register ######################################*/ 
  if(HAL_DAC_SetValue(&DacSRHandle, DAC_SR_CHANNEL2, DAC_ALIGN_12B_R, DAC_SR2_TURN_OFF_COMP_THRESHOLD) != HAL_OK) 
  {
    /* Setting value Error */
    Init_Error_Handler();        
  }
  
  /*##-4A- Enable DAC SR Channel1 ################################################*/ 
  if(HAL_DAC_Start(&DacSRHandle, DAC_SR_CHANNEL1) != HAL_OK)
  {
    /* Start Error */
    Init_Error_Handler();      
  }
  
  /*##-4B- Enable DAC SR Channel1 ################################################*/ 
  if(HAL_DAC_Start(&DacSRHandle, DAC_SR_CHANNEL2) != HAL_OK)
  {
    /* Start Error */
    Init_Error_Handler();      
  }
  
}

/**
  * @brief  Configure USART to comunicate with secondary DC/DC.
  * @param  None
  * @retval None
  */
void USART_Config(void) //for DSMPS B2B Communication
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit    = One Stop bit
      - Parity      = None parity
      - BaudRate    = User's baud rate
      - Hardware flow control disabled (RTS and CTS signals) */
//  UartHandle.Instance        = USARTx;
//  
//  UartHandle.Init.BaudRate   = USART_BAUD_RATE;
//  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
//  UartHandle.Init.StopBits   = UART_STOPBITS_1;
//  UartHandle.Init.Parity     = UART_PARITY_NONE; 
//  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
//  UartHandle.Init.Mode       = UART_MODE_TX_RX;
//  
//#ifdef OPTOCOUPLER
//  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
//  UartHandle.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
//#else
//  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//#endif
//  
//  if(HAL_UART_Init(&UartHandle) != HAL_OK)
//  {
//    /* Initialization Error */
//    Init_Error_Handler(); 
//  }
//  
//  if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)UsartRxBuffer, USARTx_RX_BUFFER_SIZE) != HAL_OK)
//  {
//    /* Transfer error in reception process */
//    Init_Error_Handler();     
//  }
  
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************** (C) COPYRIGHT STMicroelectronics *******************/
