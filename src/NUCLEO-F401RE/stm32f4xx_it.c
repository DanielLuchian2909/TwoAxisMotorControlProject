/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    13/05/2015 09:14:38
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_it.h"
#include "xnucleoihm02a1_interface.h"
#include "example_usart.h"
#include "L6470.h"

/* Defines ------------------------------------------------------------------*/
#define DEBOUNCE_TIME_ITERATIONS 200000

//X positive limit switch - Returns true if high (pressed), false if low (not pressed)
#define IsLimitSwitchXPosHigh() (HAL_GPIO_ReadPin(LIMIT_SWITCH_XPOS_PORT, LIMIT_SWITCH_XPOS_PIN) == GPIO_PIN_SET)

//X negative limit switch - Returns true if high (pressed), false if low (not pressed)
#define IsLimitSwitchXNegHigh() (HAL_GPIO_ReadPin(LIMIT_SWITCH_XNEG_PORT, LIMIT_SWITCH_XNEG_PIN) == GPIO_PIN_SET)

//Y positive limit switch - Returns true if high (pressed), false if low (not pressed)
#define IsLimitSwitchYPosHigh() (HAL_GPIO_ReadPin(LIMIT_SWITCH_YPOS_PORT, LIMIT_SWITCH_YPOS_PIN) == GPIO_PIN_SET)

//Y negative limit switch - Returns true if high (pressed), false if low (not pressed)
#define IsLimitSwitchYNegHigh() (HAL_GPIO_ReadPin(LIMIT_SWITCH_YNEG_PORT, LIMIT_SWITCH_YNEG_PIN) == GPIO_PIN_SET)

//Axis switch - Returns true if high (pressed), false if low (not pressed)
#define IsAxisSwitchHigh() (HAL_GPIO_ReadPin(AXIS_SWITCH_PORT, AXIS_SWITCH_PIN) == GPIO_PIN_SET)

//Check if MX is moving forward (returns 1) or backward (returns 0) using enum for direction
#define IsMotorXMovingForward() (L6470_CheckStatusRegisterFlag(MOTOR_X, DIR_ID) == L6470_DIR_FWD_ID)

//Check if MY is moving forward (returns 1) or backward (returns 0) using enum for direction
#define IsMotorYMovingForward() (L6470_CheckStatusRegisterFlag(MOTOR_Y, DIR_ID) == L6470_DIR_FWD_ID)


/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @addtogroup STM32F4XX_IT
  * @{
  */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
  * @addtogroup STM32F4XX_IT_Exported_Functions
  * @{
  */

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI Line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles EXTI Line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
* @brief This function handles EXTI Line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(AXIS_SWITCH_PIN) != RESET)
  {

    //1) Debounce the signal
    for (volatile uint32_t j=0; j<DEBOUNCE_TIME_ITERATIONS; j++) 
    {}
        
    //2) Clear the interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(AXIS_SWITCH_PIN); 

    //3) If interrupt is still high stop the motor and switch the current motor to the other one
    if (IsAxisSwitchHigh())
    {
      L6470_HardStop(g_curr_motor);
      g_curr_motor = !g_curr_motor;
    }
  } 
}

/**
 * @brief This function handles EXTI Line[9:5] interrupt.
 */
void EXTI9_5_IRQHandler(void)
{
  //If the limit switch for the positive x-axis (M0) is triggered, 
  //stop the motor and reverse it until the limit switch turns off
  if(__HAL_GPIO_EXTI_GET_IT(LIMIT_SWITCH_XPOS_PIN) != RESET)
  {
    //1) Debounce the signal
    for (volatile uint32_t i=0; i<DEBOUNCE_TIME_ITERATIONS; i++) 
    {}

    //2) Clear the interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_SWITCH_XPOS_PIN); 
    
    //3) If the X Pos limit switch is high, motor0 is going fwd, and the X Neg limit switch is low, go backwards
    //   Else If the X Pos limit switch is high and the X Neg limit switch is high, stop
    if ( IsLimitSwitchXPosHigh() && (!IsLimitSwitchXNegHigh()) && IsMotorXMovingForward() )
    {
      L6470_Run(MOTOR_X, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    else if (IsLimitSwitchXPosHigh() && IsLimitSwitchXNegHigh())
    {
      L6470_HardStop(MOTOR_X);
    }
  }


  //If the limit switch for the negative x-axis (M0) is triggered, 
  //stop the motor and reverse it until the limit switch turns off
  if(__HAL_GPIO_EXTI_GET_IT(LIMIT_SWITCH_XNEG_PIN) != RESET)
  {
    //1) Debounce the signal
    for (volatile uint32_t i=0; i<DEBOUNCE_TIME_ITERATIONS; i++) 
    {}

    //2) Clear the interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_SWITCH_XNEG_PIN); 

    //3) If the X Neg limit switch is high, motor0 is going backwards, and the X Pos limit switch is low, go forwards
    //   Else If the X Neg limit switch is high and the X Pos limit switch is high, stop
    if ( IsLimitSwitchXNegHigh() && (!IsLimitSwitchXPosHigh()) && (!IsMotorXMovingForward()) ) 
    {
      L6470_Run(MOTOR_X, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    else if (IsLimitSwitchXPosHigh() && IsLimitSwitchXNegHigh())
    {
      L6470_HardStop(MOTOR_X);
    }
  }


  //If the limit switch for the positive y-axis (M1) is triggered, 
  //stop the motor and reverse it until the limit switch turns off
  if(__HAL_GPIO_EXTI_GET_IT(LIMIT_SWITCH_YPOS_PIN) != RESET)
  {
    //1) Debounce the signal
    for (volatile uint32_t i=0; i<DEBOUNCE_TIME_ITERATIONS; i++) 
    {}

    //2) Clear the interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_SWITCH_YPOS_PIN); 

    //3) If the Y Pos limit switch is high, motor1 is going fwd, and the Y Neg limit switch is llow, go backwards
    //   Else If the Y Pos limit switch is high and the Y Neg limit switch is high, stop
    if ( IsLimitSwitchYPosHigh() && (!IsLimitSwitchYNegHigh()) && IsMotorYMovingForward())
    {
      L6470_Run(MOTOR_Y, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    else if ( IsLimitSwitchYPosHigh() && IsLimitSwitchYNegHigh() )
    {
      L6470_HardStop(MOTOR_Y);
    }
  }


  //If the limit switch for the negative y-axis (M1) is triggered, 
  //stop the motor and reverse it until the limit switch turns off
  if(__HAL_GPIO_EXTI_GET_IT(LIMIT_SWITCH_YNEG_PIN) != RESET)
  {
    //1) Debounce the signal 
    for (volatile uint32_t i=0; i<DEBOUNCE_TIME_ITERATIONS; i++) 
    {}

    //2) Clear the interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_SWITCH_YNEG_PIN); 

    //3) If the Y Neg limit switch is high, motor1 is going backwards, and the Y Pos limit switch is low, go forwards
    //   Else If the Y Neg limit switch is high and the Y Pos limit switch is high, stop
    if ( IsLimitSwitchYNegHigh() && (!IsLimitSwitchYPosHigh()) && (!IsMotorYMovingForward()) )
    {
      L6470_Run(MOTOR_Y, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    else if (IsLimitSwitchYPosHigh() && IsLimitSwitchYNegHigh())
    {
      L6470_HardStop(MOTOR_Y);
    }
  } 
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
  USART_ITCharManager(&huart2);
}

/**
* @brief This function handles EXTI Line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @}
  */ /* End of STM32F4XX_IT_Exported_Functions */

/**
  * @}
  */ /* End of STM32F4XX_IT */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
