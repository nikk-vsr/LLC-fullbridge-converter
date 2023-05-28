/**
  @page Project_Description
  @verbatim
  
  ******************** (C) COPYRIGHT STMicroelectronics *************+++++******
  * @file    readme.c 
  * @author  System Lab
  * @version V1.6.0
  * @date    31-May-2018
  * @brief   Description of 3kW DSMPS Full Bridge LLC Firmware
  ******************************************************************************
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
  
  @endverbatim

@par Firmware Version
  DSMPS LLC FW V1.6 
  
@par Change Log
	- V1.5:	• First official release

        - V1.6:	• HRTIM_TIMA used for synchronizzation instead of HRTIM_MASTER
                • More stable control loop
                • Adaptive SR retuned
                • Reviewed light load burst mode
                • DAC_OVERCURRENT_COMP_THRESHOLD and OUT_CURRENT_MAX retuned
                • Current sensor offset calibration
                • Minimum fan speed decreased
                • If hard fault occurs, FAULT_LED is kept ON
                • LED_DCDC_OUT_UNDER_VOLTAGE_BLINK_NUM changed from 3 to 2
                • LED_DCDC_OUT_OVER_VOLTAGE_BLINK_NUM changed from 2 to 3
                • AWD on Vout disabled by default

	
@par Firmware Description 
  The firmware is designed for STM32F334R8 microcontroller with IAR Embedded Workbench toolchain and it is provided with the digital control board STEVAL- DPS334C1.
  The primary side MOSFETs are driven by 50% duty cycle PWM signals, with a proper dead time for each leg to ensure ZVS operation and avoid input voltage shot-through.
  The PI voltage regulator, executed at 50 kHz, provides the PWM switching period of primary side devices, in order to change the voltage gain of the resonant tank and then, 
  to regulate the output voltage at the desired value of 48V.
  At board’s power on, a frequency ramp-up, starting from 380 kHz, is performed to avoid current spikes.
  An adaptive SR algorithm, based on Vds sensing, is used to drive the secondary side MOSFETs and reduce conduction losses.
  A burst mode operation is adopted for light loads to reduce the switching losses and increase the converter’s efficiency.
  The control FW provides also a fast overcurrent protection, input and output under/over voltage protections and an over-temperature protection.
  An additional PWM signal is used to drive the fan placed on the power board, with a speed that depends on both output load and heatsink temperature.
  At last, the serial user interface  allows to change the main parameters of the converter and enable/disable each control feature, such as the open loop operation.


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 
  - FullBridge_LLC_Project/EWARM/Inc/System & Drive Params/LLC_control_prm.h            Control parameters, frequencies and user defines for each modality 
  - FullBridge_LLC_Project/EWARM/Inc/System & Drive Params/LLC_board_config_param.h     Definitions of input/output ports and conversion factors
  - FullBridge_LLC_Project/EWARM/Inc/System & Drive Params/HRTIM_pwm_config_param.h     Definitions used for HRTIM configuration
  - FullBridge_LLC_Project/EWARM/Inc/Control_Layer.h           			        Prototypes of control layer functions
  - FullBridge_LLC_Project/EWARM/Inc/Fault_Processing.h    			        Prototypes of fault detection functions
  - FullBridge_LLC_Project/EWARM/Inc/LLC_Globals.h    				        Declarations of the exported variables of module "LLC_Globals.c"
  - FullBridge_LLC_Project/EWARM/Inc/LLC_Init_Periph.h    			        Prototypes of init peripherals functions
  - FullBridge_LLC_Project/EWARM/Inc/LLC_PWMnCurrVoltFdbk.h    			        Prototypes of PWM drive and acquisition functions
  - FullBridge_LLC_Project/EWARM/Inc/DSMPS_type.h    				        Type definitions for generic DSMPS control
  - FullBridge_LLC_Project/EWARM/Inc/LLC_type.h    				        Type definitions of LLC control
  - FullBridge_LLC_Project/EWARM/Inc/UI_UART_Interface.h    			        Prototypes of UI Communication functions and related parameters
  - FullBridge_LLC_Project/EWARM/Inc/Digital_Filters.h				        Prototypes of digital filter functions  
  - FullBridge_LLC_Project/EWARM/Inc/PID_regulators.h    			        Prototypes of PI(D) and related functions
  - FullBridge_LLC_Project/EWARM/Inc/StateMachine.h                                     Prototypes of state machine related functions
  - FullBridge_LLC_Project/EWARM/Inc/stm32f3xx_hal_conf.h    			        HAL configuration file  
  - FullBridge_LLC_Project/EWARM/Inc/stm32f3xx_it.h         			        PPP interrupt handlers header file
  - FullBridge_LLC_Project/EWARM/Src/Control_Layer.c           			        Functions power stage
  - FullBridge_LLC_Project/EWARM/Src/Fault_Processing.c    			        Fault detection functions and related thresholds
  - FullBridge_LLC_Project/EWARM/Src/LLC_Globals.c    				        Global variables for the control of battery charger  
  - FullBridge_LLC_Project/EWARM/Src/LLC_Init_Periph.c    			        Init peripherals functions  
  - FullBridge_LLC_Project/EWARM/Src/LLC_PWMnCurrVoltFdbk.c    			        PWM drive and acquisition functions
  - FullBridge_LLC_Project/EWARM/Src/UI_UART_Interface.c    			        User Interface Serial Communication and related functions
  - FullBridge_LLC_Project/EWARM/Src/main.c    				  	        Main program and state machine
  - FullBridge_LLC_Project/EWARM/Src/Digital_Filters.c				        Digital filter functions  
  - FullBridge_LLC_Project/EWARM/Src/PID_regulators.c    			        PI(D) regulators and related functions
  - FullBridge_LLC_Project/EWARM/Src/stm32f3xx_hal_msp.c     			        HAL MSP file
  - FullBridge_LLC_Project/EWARM/Src/stm32f3xx_it.c          			        PPP interrupt handlers
  
@par Hardware and Software environment 

  - This example runs on STM32F334x4/STM32F334x6/STM32F334x8 devices.
    
  - This example has been tested with STM32F3348-DISCO
    board and can be easily tailored to any other supported device and development board.
  
  - STM32F3348-DISCO Set-up: PA7 must be briefly tied to Vdd to simulate a fault event

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
