/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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

//test git

 
#include "example.h"
#include "example_usart.h"
#include "L6470.h"

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/* Function Prototypes */
static void GPIO_LimitSwitch_Init(void);

/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */

volatile uint8_t PA8IT = 0;
volatile uint8_t PA9IT = 0;
volatile int PA8V;
volatile int PA9V;

volatile uint8_t PB6IT = 0;
volatile uint8_t PC7IT = 0;
volatile int PB6V;
volatile int PC7V;

int main(void)
{
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();
  
#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
  	USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
#endif
  
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  
  /* Infinite loop */
  while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();

  /* Enable the GPIOA, B, C Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Init the GPIO pins for the limit switching*/
  GPIO_LimitSwitch_Init();

  /* Configure EXTI Line[9:5] interrupt priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);

  /* Enable EXTI Line[9:5] interrupt */
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  //L6470_Run(0, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
  //L6470_Run(1, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
 
  //L6470_Run(0, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED); 
  //L6470_Run(1, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);

  /* Infinite loop */
  while (1)
  {
    PA8V = HAL_GPIO_ReadPin(LIMIT_SWITCH_XPOS_PORT, LIMIT_SWITCH_XPOS_PIN);
    PA9V = HAL_GPIO_ReadPin(LIMIT_SWITCH_XNEG_PORT, LIMIT_SWITCH_XNEG_PIN);
    PB6V = HAL_GPIO_ReadPin(LIMIT_SWITCH_YPOS_PORT, LIMIT_SWITCH_YPOS_PIN);
    PC7V = HAL_GPIO_ReadPin(LIMIT_SWITCH_YNEG_PORT, LIMIT_SWITCH_YNEG_PIN);

    /* Check if any Application Command for L6470 has been entered by USART */
   // USART_CheckAppCmd();
  }
#endif
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @brief Initiliaze the interrupt lines for the limit switches
 * @param None
 * @retval None 
 */
static void GPIO_LimitSwitch_Init(void)
{  
  /* Configure limit switch positive x-axis (M0) GPIO using pin PA8 */
  GPIO_InitTypeDef GPIO_InitStruct_LimitSwitch_XPos;
  GPIO_InitStruct_LimitSwitch_XPos.Pin = LIMIT_SWITCH_XPOS_PIN; 
  GPIO_InitStruct_LimitSwitch_XPos.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_LimitSwitch_XPos.Pull = GPIO_PULLUP;
  GPIO_InitStruct_LimitSwitch_XPos.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LIMIT_SWITCH_XPOS_PORT, &GPIO_InitStruct_LimitSwitch_XPos);

  /* Configure limit switch negative x-axis (M0) GPIO using pin PA9 */
  GPIO_InitTypeDef GPIO_InitStruct_LimitSwitch_XNeg;
  GPIO_InitStruct_LimitSwitch_XNeg.Pin = LIMIT_SWITCH_XNEG_PIN; 
  GPIO_InitStruct_LimitSwitch_XNeg.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_LimitSwitch_XNeg.Pull = GPIO_PULLUP;
  GPIO_InitStruct_LimitSwitch_XNeg.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LIMIT_SWITCH_XNEG_PORT, &GPIO_InitStruct_LimitSwitch_XNeg);

  /* Configure limit switch positive y-axis (M1) GPIO using pin PB6 */
  GPIO_InitTypeDef GPIO_InitStruct_LimitSwitch_YPos;
  GPIO_InitStruct_LimitSwitch_YPos.Pin = LIMIT_SWITCH_YPOS_PIN; 
  GPIO_InitStruct_LimitSwitch_YPos.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_LimitSwitch_YPos.Pull = GPIO_PULLUP;
  GPIO_InitStruct_LimitSwitch_YPos.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LIMIT_SWITCH_YPOS_PORT, &GPIO_InitStruct_LimitSwitch_YPos);

  /* Configure limit switch negative x-axis (M1) GPIO using pin PB7 */
  GPIO_InitTypeDef GPIO_InitStruct_LimitSwitch_YNeg;
  GPIO_InitStruct_LimitSwitch_YNeg.Pin = LIMIT_SWITCH_YNEG_PIN; 
  GPIO_InitStruct_LimitSwitch_YNeg.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_LimitSwitch_YNeg.Pull = GPIO_PULLUP;
  GPIO_InitStruct_LimitSwitch_YNeg.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LIMIT_SWITCH_YNEG_PORT, &GPIO_InitStruct_LimitSwitch_YNeg);
}

/**
 * @brief Testing Limit Switch Function
 * @return //1 for successful, it would be stuck for unsuccessful (some sort of timeout then return 0?)
 * @param //Which motor to test
 */
uint8_t LimitSwitch_Test(uint8_t L6470_Id)
{
  //Run the motor forward
  L6470_Run(L6470_Id, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);

  //Tight poll until motor starts reverse direction (after limit switch is hit)
  while (L6470_CheckStatusRegisterFlag(L6470_Id, DIR_ID)){}

  HAL_Delay(100);

  //Run the motor backwards
  L6470_Run(L6470_Id, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);

  //Tight poll until the motor starts going in the forward direction (after limit switch is hit)
  while (!L6470_CheckStatusRegisterFlag(L6470_Id, DIR_ID)){}

  //If it reached here limit switches successfully ran
  return 1;
}

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
