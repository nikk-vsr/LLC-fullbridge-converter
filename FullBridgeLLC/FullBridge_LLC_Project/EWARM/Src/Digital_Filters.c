/**
  ******************************************************************************
  * @file    Digital_Filters.c
  * @author  IMS Systems Lab 
  * @version V1.0.0
  * @date    16-Dec-2014
  * @brief   This file provides fuctions to perform several types of digital filters
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

/* Standard include ----------------------------------------------------------*/
#include "stm32f3xx_hal.h" 
#include "arm_math.h"
#include "Digital_Filters.h"

/** @addtogroup DSMPS_project
  * @{
  */

/** @addtogroup Digital_Filters
  * @{
  */

/* Private Variables ----------------------------------------------------------*/
/***IIR filter and Variables 12th order with cascade of 6 second order stages ***/
arm_biquad_casd_df1_inst_f32 IIR_Struct;        /**< Instance structure for the floating-point Biquad cascade filter */
float32_t IIR_FilterStateBuffF32[24];          /**< array of state coefficients.  The array is of length 4*numStages */

//const float32_t IIR_FilterCoeffBuffF32[30] = { /**< array of state coefficients.  The array is of length 4*numStages */
//0.895717308331452800, -1.685526379656990150, 0.895717308331461015, 1.682127847529434560, -0.807319154825458574,
//0.923669070596318642, -1.738124930803821800, 0.923669070596310426, 1.740121135895487340, -0.832512363528298272,
//0.903487390040318306, -1.700147820563139290, 0.903487390040326743, 1.675149223792099070, -0.840513972504363660,
//0.957801495032364114, -1.802354014303139570, 0.957801495032355565, 1.817882613100834990, -0.891042363551229011,
//0.956214639007693990, -1.799367929669668520, 0.956214639007702871, 1.736169014354570490, -0.935637612225140747,
//0.983237582886694939, -1.850218666102280360, 0.983237582886686168, 1.896183373953512110, -0.962079042480238678,
//};


/**
  * @brief  performs digital filter of a not filtered variable (unsigned16)
  * @param  hNoFilteredValue not filtered input variable
  * @param  *phLastFilteredValue pointer to last filtered variable
  * @param  bDigitShift order of filter (n in the following formula)
  * @retval hFilteredValue  filtered value of input varible 
  * 
  * The digital filter applies the formula: yk = y(k-1) - (y(k-1)>>n) + (xk>>n)  
*/
#pragma location = ".ccmram"
uint16_t uSimpleDigitalLowPassFilter(uint16_t hNoFilteredValue, uint16_t *phLastFilteredValue, uint8_t bDigitShift)
{
  uint16_t hFilteredValue = *phLastFilteredValue;
  uint32_t wAux = 0;
  
  /* yk = y(k-1) - (y(k-1)>>n) + (xk>>n) */
//  filteredValue = filteredValue - (filteredValue >> digitShift) + (noFilteredValue >> digitShift);
  wAux = (hFilteredValue << bDigitShift) - hFilteredValue + hNoFilteredValue;
  wAux = (wAux >> bDigitShift);
  hFilteredValue = (uint16_t)wAux;
//  *phLastFilteredValue = hFilteredValue;           // no update: the new value is returned by function
  
  return hFilteredValue;
}

/**
  * @brief  performs digital filter of a not filtered variable (signed16)
  * @param  hNoFilteredValue not filtered input variable
  * @param  *phLastFilteredValue pointer to last filtered variable
  * @param  bDigitShift order of filter (n in the following formula)
  * @retval hFilteredValue  filtered value of input varible 
  * 
  * The digital filter applies the formula: yk = y(k-1) - (y(k-1)>>n) + (xk>>n)  
*/
#pragma location = ".ccmram"
int16_t sSimpleDigitalLowPassFilter(int16_t hNoFilteredValue, int16_t *phLastFilteredValue, uint8_t bDigitShift)
{
  int16_t hFilteredValue = *phLastFilteredValue;
  int32_t wAux = 0;
  
  /* yk = y(k-1) - (y(k-1)>>n) + (xk>>n) */
//  filteredValue = filteredValue - (filteredValue >> digitShift) + (noFilteredValue >> digitShift);
//  wAux = (hFilteredValue << bDigitShift) - hFilteredValue + hNoFilteredValue;
  
  wAux = (hFilteredValue << bDigitShift);
  wAux -= hFilteredValue;
  wAux += hNoFilteredValue;
  wAux = (wAux >> bDigitShift);
  hFilteredValue = (int16_t)wAux;
  
  return hFilteredValue;
  
//  wAux = (hFilteredValue << bDigitShift) - hFilteredValue + hNoFilteredValue;
//  wAux = (wAux >> bDigitShift);
//  return (int16_t)wAux;
}

/**
  * @brief  performs IIR band stop digital filter of a not filtered variable (unsigned16)
  * @param  hNoFilteredValue not filtered input variable
  * @retval fFilteredValue filtered value of input varible 
  *   
*/
uint16_t IIR_DigitalFilter(uint16_t hNoFilteredValue)   /* band stop filter using IIR filter definition 
{
  float32_t fNoFilteredValue = (float32_t)hNoFilteredValue;
  float32_t fFilteredValue;
  
  // execute arm digital filter function
  arm_biquad_cascade_df1_f32(&IIR_Struct, &fNoFilteredValue, &fFilteredValue, 1);
  
  return ((uint16_t)fFilteredValue);  
}
  		   
/**
  * @}
  */ 

/**
  * @}
  */ 

/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************/
