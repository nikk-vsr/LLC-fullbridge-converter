/**
  ******************************************************************************
  * @file    PID_regulators.c
  * @author  Systems Lab 
  * @version V1.3.0
  * @date    24-May-2017
  * @brief   This file provides fuctions to perform PID control
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

/* Standard include ----------------------------------------------------------*/
#include "PID_regulators.h"
#include "LLC_control_param.h"

/** @addtogroup DSMPS_project
  * @{
  */

/** @addtogroup General_PID_functions
  * @{
  */

/**
  * @brief  Set PID proportional gain
  * @param  PID_Struct: pointer to PID structure
  * @param  hProportionalGain: value of proportional gain
  * @retval None 
  * 
  */
void PID_Set_ProportionalGain(PID_Struct_t *PID_Struct, int16_t hProportionalGain){
  PID_Struct->hKp_Gain = hProportionalGain;
}

/**
  * @brief  Set PID integral gain
  * @param  PID_Struct: pointer to PID structure
  * @param  hIntegralGain: value of integral gain
  * @retval None 
  * 
  */
void PID_Set_IntegralGain(PID_Struct_t *PID_Struct, int16_t hIntegralGain){
  PID_Struct->hKi_Gain = hIntegralGain;
}

/**
  * @brief  Reset PID integral terms for Passed PI object
  * @param  PID_Struct to PI structure to reset
  * @retval None 
  * 
  */
void PID_Reset_IntegralTerm(PID_Struct_t *PID_Struct){
  PID_Struct->wIntegral = 0;
}

/**
  * @brief  Set PID integral term for Passed PI object to a desired value
  * @param  PID_Struct: pointer to PI structure 
  * @param  wIntegralTerm: value of integral term without considering ki divisor   
  * @retval None 
  * 
  */
void PID_Set_IntegralTerm(PID_Struct_t *PID_Struct, int32_t wIntegralTerm){
  int64_t dIntegral_tmp = (((int64_t)wIntegralTerm)*(PID_Struct->hKi_Divisor));
  
  if (dIntegral_tmp > (PID_Struct->wUpper_Limit_Integral))  //wUpper_Limit_Integral - control input upper limit
  {
    dIntegral_tmp = PID_Struct->wUpper_Limit_Integral;
  }
  else if (dIntegral_tmp < (PID_Struct->wLower_Limit_Integral))
  {
    dIntegral_tmp = PID_Struct->wLower_Limit_Integral;
  }
  
  PID_Struct->wIntegral = (int32_t)dIntegral_tmp;
}

/**
  * @brief  Set PID output upper limit
  * @param  PID_Struct: pointer to PID structure
  * @param  hOutputUpperLimit: value of new output upper limit             ( actuator limit )
  * @retval None 
  * 
  */
void PID_Set_OutputUpperLimit(PID_Struct_t *PID_Struct, int16_t hOutputUpperLimit){
  PID_Struct->hUpper_Limit_Output = hOutputUpperLimit;
}

/**
  * @brief  Set PID output lower limit
  * @param  PID_Struct: pointer to PID structure
  * @param  hOutputLowerLimit: value of new output lower limit
  * @retval None 
  * 
  */
void PID_Set_OutputLowerLimit(PID_Struct_t *PID_Struct, int16_t hOutputLowerLimit){
  PID_Struct->hLower_Limit_Output = hOutputLowerLimit;
}

/**
  * @brief  Compute the PI(D) output for a PI(D) regulation.
  * 
  *     - Kp_Gain: proportional coeffcient 
  *     - Ki_Gain: integral coeffcient 
  *     - Kd_Gain: differential coeffcient
  * 
  * @param  *PID_Struct to Pointer to the PID settings 
  * @param  hReference reference value for control loop
  * @param  hPresentFeedback reference value for control loop
  * @retval int16_t output of PID regulator
  * 
  */
#pragma location = ".ccmram"    
int16_t PID_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_t *PID_Struct)
 {
   int32_t wError, wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
   int32_t wDischarge = 0;
   int16_t hUpperOutputLimit = PID_Struct->hUpper_Limit_Output;
   int16_t hLowerOutputLimit = PID_Struct->hLower_Limit_Output;

 #ifdef DIFFERENTIAL_TERM_ENABLED    
   int32_t wDifferential_Term;
 #endif    
   // error computation
   wError= (int32_t)(hReference - hPresentFeedback);
  
   // Proportional term computation
   wProportional_Term = PID_Struct->hKp_Gain * wError;

   // Integral term computation
   if (PID_Struct->hKi_Gain == 0)
   {
     PID_Struct->wIntegral = 0;
   }
   else
   { 
     wIntegral_Term = PID_Struct->hKi_Gain * wError;
     wIntegral_sum_temp = PID_Struct->wIntegral + wIntegral_Term;
     
     if (wIntegral_sum_temp < 0)
     {
      if (PID_Struct->wIntegral > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = S32_MAX;
        }
      }
    }
    else
    {
      if (PID_Struct->wIntegral < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = -S32_MAX;
        }
      }
    }	
    
     if (wIntegral_sum_temp > PID_Struct->wUpper_Limit_Integral)
     {
       PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
     }
     else if (wIntegral_sum_temp < PID_Struct->wLower_Limit_Integral)
     { 
       PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
     }
     else
     {
      PID_Struct->wIntegral = wIntegral_sum_temp;
     }
    
   }
  // Differential term computation
 #ifdef DIFFERENTIAL_TERM_ENABLED
   {
   int32_t wtemp;
  
   wtemp = wError - PID_Struct->wPreviousError;
   wDifferential_Term = PID_Struct->hKd_Gain * wtemp;
   PID_Struct->wPreviousError = wError;    // store value 
   }
   wOutput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                 PID_Struct->wIntegral/PID_Struct->hKi_Divisor + 
                 wDifferential_Term/PID_Struct->hKd_Divisor); 

 #else  
   wOutput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                 PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
 #endif

  if (wOutput_32 > hUpperOutputLimit)
  {    
    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;		  			 	
  }
  else if (wOutput_32 < hLowerOutputLimit)
  {    
    wDischarge = hLowerOutputLimit - wOutput_32; 
    wOutput_32 = hLowerOutputLimit;
  }
  else
  {}
  
  PID_Struct->wIntegral += wDischarge;
  
  return((int16_t)(wOutput_32));  
 }

/**
  * @brief  Compute the PI(D) output for a PI(D) regulation. (Custom PID)
  * 
  *     - Kp_Gain: proportional coeffcient 
  *     - Ki_Gain: integral coeffcient 
  *     - Kd_Gain: differential coeffcient
  * 
  * @param  *PID_Struct to Pointer to the PID settings 
  * @param  hReference reference value for control loop
  * @param  hPresentFeedback reference value for control loop
  * @retval int32_t output of PID regulator
  * 
  */
int32_t PID_Controller(int16_t hReference, int16_t hPresentFeedback, PID_Struct_t *PID_Struct){
  
  int32_t wError, wProportionalOut, wIntegralOut, wTmpIntegralTerm, wTotalOutput;

  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);
  
  // Proportional term computation
  wProportionalOut = PID_Struct->hKp_Gain * wError;

  // Integral term computation
  if (PID_Struct->hKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  { 
    wTmpIntegralTerm = (int32_t)((PID_Struct->hKi_Gain * wError)/PID_Struct->hKi_Divisor);
    wIntegralOut = PID_Struct->wIntegral + (int32_t)(wTmpIntegralTerm);
    
    /*** avoid overflow ***/
    if ((wTmpIntegralTerm >= 0) && (PID_Struct->wIntegral >= 0)){
            if(wIntegralOut < 0){
              wIntegralOut = S32_MAX;
              wTotalOutput = S32_MAX;	// if integralOut saturates, totalOut goes in overflow (prop term has same sign of tmpIntegralTerm)
            }
    }
	  
    /*** avoid underflow ***/
    else if((wTmpIntegralTerm <= 0) && (PID_Struct->wIntegral <= 0)){
            if(wIntegralOut > 0){
              wIntegralOut = S32_MIN;
              wTotalOutput = S32_MIN;	// if integralOut saturates, totalOut goes in underflow (prop term has same sign of tmpIntegralTerm)
            }	
    }
  }
  if((wIntegralOut != S32_MIN)&&(wIntegralOut != S32_MAX)){	//if integralOut = (S32_MAX || S32_MIN), totalOut was saturated
     wTotalOutput = wProportionalOut + wIntegralOut;		        /* is supposed that proportionalOut < S32_MAX - integralOut
                                                                  and proportionalOut > S32_MIN - integralOut.
                                                                  it is so if parameters, upperLimit and lowerLimit are chosen properly */
  }
  
  /*** integration with anti-windup ***/
  if (wTotalOutput > PID_Struct->hUpper_Limit_Output) { 
    return(PID_Struct->hUpper_Limit_Output);
  }
  else if (wTotalOutput < PID_Struct->hLower_Limit_Output){	
    return(PID_Struct->hLower_Limit_Output);
  }
  else{           
     PID_Struct->wIntegral  = wIntegralOut;   // if (totalOut<upperLimit)&&(totalOut>lowerLimit) update wIntegral
  }
      
  return wTotalOutput;  
}
  		 
/**
  * @}
  */ 

/** @addtogroup General_PID_extended_functions
  * @{
  */

/**
  * @brief  Set PID proportional gain for extended PID
  * @param  PID_Struct: pointer to extended PID structure
  * @param  hProportionalGain: value of proportional gain
  * @retval None 
  * 
  */
void PID_Set_ProportionalGain_ex(PID_Struct_ex_t *PID_Struct, int16_t hProportionalGain){
  PID_Struct->hKp_Gain = hProportionalGain;
}

/**
  * @brief  Set PID integral gain for extended PID
  * @param  PID_Struct: pointer to extended PID structure
  * @param  hIntegralGain: value of integral gain
  * @retval None 
  * 
  */
void PID_Set_IntegralGain_ex(PID_Struct_ex_t *PID_Struct, int16_t hIntegralGain){
  PID_Struct->hKi_Gain = hIntegralGain;
}

/**
  * @brief  Reset PID integral terms for Passed PI object
  * @param  PID_Struct: pointer to extended PI structure to reset
  * @retval None 
  * 
  */
void PID_Reset_IntegralTerm_ex(PID_Struct_ex_t *PID_Struct){
  PID_Struct->wIntegral = 0;
}

/**
  * @brief  Set PID integral terms for Passed PI object to a desired value
  * @param  PID_Struct: pointer to extended PI structure to reset
  * @param  wIntegralTerm: value of integral term without considering Ki divisor
  * @retval None 
  * 
  */
//void PID_Set_IntegralTerm_ex(PID_Struct_ex_t *PID_Struct, int32_t wIntegralTerm){
//  PID_Struct->wIntegral = (wIntegralTerm*(PID_Struct->hKi_Divisor));
//}

void PID_Set_IntegralTerm_ex(PID_Struct_ex_t *PID_Struct, int32_t wIntegralTerm){
  int64_t dIntegral_tmp = (((int64_t)wIntegralTerm)*(PID_Struct->hKi_Divisor));
  
  if (dIntegral_tmp > (PID_Struct->wUpper_Limit_Integral))
  {
    dIntegral_tmp = PID_Struct->wUpper_Limit_Integral;
  }
  else if (dIntegral_tmp < (PID_Struct->wLower_Limit_Integral))
  {
    dIntegral_tmp = PID_Struct->wLower_Limit_Integral;
  }
  
  PID_Struct->wIntegral = (int32_t)dIntegral_tmp;
}
  
/**
  * @brief  Set PID output upper limit
  * @param  PID_Struct: pointer to extended PID structure
  * @param  wOutputUpperLimit: value of new output upper limit
  * @retval None 
  * 
  */
void PID_Set_OutputUpperLimit_ex(PID_Struct_ex_t *PID_Struct, int32_t wOutputUpperLimit){
  PID_Struct->wUpper_Limit_Output = wOutputUpperLimit;
}

/**
  * @brief  Set PID output lower limit
  * @param  PID_Struct: pointer to extended PID structure
  * @param  wLowerUpperLimit: value of new output lower limit
  * @retval None 
  * 
  */
void PID_Set_OutputLowerLimit_ex(PID_Struct_ex_t *PID_Struct, int32_t wOutputLowerLimit){
  PID_Struct->wLower_Limit_Output = wOutputLowerLimit;
}

/**
  * @brief  Compute the PI(D) output for a PI(D) regulation with an int32_t output.
  * 
  *     - Kp_Gain: proportional coeffcient 
  *     - Ki_Gain: integral coeffcient 
  *     - Kd_Gain: differential coeffcient
  * 
  * @param  *PID_Struct to Pointer to the extended PID settings 
  * @param  hReference reference value for control loop
  * @param  hPresentFeedback reference value for control loop
  * @retval int32_t output of PID regulator
  * 
  */
#pragma location = ".ccmram"
int32_t PID_Regulator_ex(int16_t hReference, int16_t hPresentFeedback, PID_Struct_ex_t *PID_Struct)
 {
   int32_t wError, wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
   int32_t wDischarge = 0;
   int32_t wUpperOutputLimit = PID_Struct->wUpper_Limit_Output;
   int32_t wLowerOutputLimit = PID_Struct->wLower_Limit_Output;
   
 #ifdef DIFFERENTIAL_TERM_ENABLED_EX    
   int32_t wDifferential_Term;
 #endif    
   // error computation
   wError= (int32_t)(hReference - hPresentFeedback);
  
   // Proportional term computation
   wProportional_Term = PID_Struct->hKp_Gain * wError;

   // Integral term computation
   if (PID_Struct->hKi_Gain == 0)
   {
     PID_Struct->wIntegral = 0;
   }
   else
   { 
     wIntegral_Term = PID_Struct->hKi_Gain * wError;
     wIntegral_sum_temp = PID_Struct->wIntegral + wIntegral_Term;
     
if (wIntegral_sum_temp < 0)
     {
      if (PID_Struct->wIntegral > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = S32_MAX;
        }
      }
    }
    else
    {
      if (PID_Struct->wIntegral < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = -S32_MAX;
        }
      }
    }	
    
     if (wIntegral_sum_temp > PID_Struct->wUpper_Limit_Integral)
     {
       PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
     }
     else if (wIntegral_sum_temp < PID_Struct->wLower_Limit_Integral)
     { 
       PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
     }
     else
     {
      PID_Struct->wIntegral = wIntegral_sum_temp;
     }

   }
  // Differential term computation
 #ifdef DIFFERENTIAL_TERM_ENABLED_EX
   {
   int32_t wtemp;
  
   wtemp = wError - PID_Struct->wPreviousError;
   wDifferential_Term = PID_Struct->hKd_Gain * wtemp;
   PID_Struct->wPreviousError = wError;    // store value of the error to previous error          
   }
   wOutput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                 PID_Struct->wIntegral/PID_Struct->hKi_Divisor + 
                 wDifferential_Term/PID_Struct->hKd_Divisor); 

 #else  
   wOutput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                 PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
 #endif
  
  if (wOutput_32 > wUpperOutputLimit)
  {    
    wDischarge = wUpperOutputLimit - wOutput_32;
    wOutput_32 = wUpperOutputLimit;		  			 	
  }
  else if (wOutput_32 < wLowerOutputLimit)
  {    
    wDischarge = wLowerOutputLimit - wOutput_32; 
    wOutput_32 = wLowerOutputLimit;
  }
  else
  {}
  
  PID_Struct->wIntegral += wDischarge;
  
  return wOutput_32; 
  
 }

/**
  * @}
  */ 
 
 /** @addtogroup DCDC_Specific_PID_functions
  * @{
  */

#ifndef USE_EXTENDED_PID
/**
  * @brief  Initializes DCDC PID coefficients for Vbus regulator:
  * 
  *     - Kp_Gain: proportional coeffcient 
  *     - Ki_Gain: integral coeffcient 
  *     - Kd_Gain: differential coeffcient
  * 
  * @param  PID_Vout to out voltage PID structure 
  * @retval None
  *
  */
void DCDC_PID_Init(PID_Struct_t *PID_Vout)
{
   /* Vbus PID init */
   PID_Vout->hKp_Gain     = PID_VOUT_KP_DEFAULT;
   PID_Vout->hKp_Divisor  = VOUT_KPDIV2; 
   PID_Vout->hKi_Gain     = PID_VOUT_KI_DEFAULT;
   PID_Vout->hKi_Divisor  = VOUT_KIDIV2;  
   PID_Vout->hKd_Gain     = PID_VOUT_KD_DEFAULT;
   PID_Vout->hKd_Divisor  = VOUT_KDDIV2;
   PID_Vout->wPreviousError = 0;  
   PID_Vout->hLower_Limit_Output = PID_VOUT_LOWER_LIMIT; 
   PID_Vout->hUpper_Limit_Output = PID_VOUT_UPPER_LIMIT;              
   PID_Vout->wLower_Limit_Integral = PID_VOUT_INTEGRAL_LOWER_LIMIT; 
   PID_Vout->wUpper_Limit_Integral = PID_VOUT_INTEGRAL_UPPER_LIMIT;
   PID_Vout->wIntegral = 0;
}
#else
/**
  * @brief  Initializes DCDC PID coefficients for extended Vbus regulator:
  * 
  *     - Kp_Gain: proportional coeffcient 
  *     - Ki_Gain: integral coeffcient 
  *     - Kd_Gain: differential coeffcient
  * 
  * @param  PID_Vout to out voltage PID extended structure 
  * @retval None
  *
  */
void DCDC_PID_Init_ex(PID_Struct_ex_t *PID_Vout)
{
   /* Vbus PID init */
   PID_Vout->hKp_Gain     = PID_VOUT_KP_DEFAULT;
   PID_Vout->hKp_Divisor  = VOUT_KPDIV2; 
   PID_Vout->hKi_Gain     = PID_VOUT_KI_DEFAULT;
   PID_Vout->hKi_Divisor  = VOUT_KIDIV2;  
   PID_Vout->hKd_Gain     = PID_VOUT_KD_DEFAULT;
   PID_Vout->hKd_Divisor  = VOUT_KDDIV2;
   PID_Vout->wPreviousError = 0;  
   PID_Vout->wLower_Limit_Output = PID_VOUT_LOWER_LIMIT; 
   PID_Vout->wUpper_Limit_Output = PID_VOUT_UPPER_LIMIT;              
   PID_Vout->wLower_Limit_Integral = PID_VOUT_INTEGRAL_LOWER_LIMIT; 
   PID_Vout->wUpper_Limit_Integral = PID_VOUT_INTEGRAL_UPPER_LIMIT;
   PID_Vout->wIntegral = 0;
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************** (C) COPYRIGHT 2017 STMicroelectronics *******************/
