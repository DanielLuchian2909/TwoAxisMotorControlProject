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

// test git

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
 */
/* End of BSP */

/**
 * @addtogroup MicrosteppingMotor_Example
 * @{
 */

/**
 * @defgroup   ExampleTypes
 * @{
 */

// #define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE //!< Uncomment to performe the USART example
#if ((defined(MICROSTEPPING_MOTOR_EXAMPLE)) && (defined(MICROSTEPPING_MOTOR_USART_EXAMPLE)))
#error "Please select an option only!"
#elif ((!defined(MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined(MICROSTEPPING_MOTOR_USART_EXAMPLE)))
#error "Please select an option!"
#endif
#if (defined(MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined(NUCLEO_USE_USART)))
#error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

uint16_t adc_offset = 0; // Global variable to store offset correction
float adc_gain = 1.0;

/* Function Prototypes */
static void GPIO_LimitSwitch_Init(void);
static uint8_t LimitSwitch_Test(uint8_t L6470_Id);
/**
 * @}
 */
/* End of ExampleTypes */

/**
 * @brief The FW main module
 */
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

#if defined(MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();

  /* Infinite loop */
  while (1)
    ;
#elif defined(MICROSTEPPING_MOTOR_USART_EXAMPLE)
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

  /* Test Limit Switches */
  if (!LimitSwitch_Test(0))
  {
    USART_Transmit(&huart2, "Limit Switch Test Motor 0 Failed\n\r");
    return 0;
  }

  if (!LimitSwitch_Test(1))
  {
    USART_Transmit(&huart2, "Limit Switch Test Motor 1 Failed\n\r");
    return 0;
  }

  ADC_Calculate_Gain(0.5);
  // Wait for us to check value

  ADC_Calculate_Gain(1.65);
  // Wait for us to check value

  ADC_Calculate_Gain(3.0);
  // Wait for us to check value

  /* Infinite loop */
  while (1)
  {
    /* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd();
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
void assert_failed(uint8_t *file, uint32_t line)
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
  GPIO_InitTypeDef GPIO_InitStruct_PA8;
  GPIO_InitStruct_PA8.Pin = GPIO_PIN_8;
  GPIO_InitStruct_PA8.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct_PA8.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_PA8.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PA8);

  /* Configure limit switch negative x-axis (M0) GPIO using pin PA9 */
  GPIO_InitTypeDef GPIO_InitStruct_PA9;
  GPIO_InitStruct_PA9.Pin = GPIO_PIN_9;
  GPIO_InitStruct_PA9.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct_PA9.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_PA9.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PA9);

  /* Configure limit switch positive y-axis (M1) GPIO using pin PB6 */
  GPIO_InitTypeDef GPIO_InitStruct_PB6;
  GPIO_InitStruct_PB6.Pin = GPIO_PIN_6;
  GPIO_InitStruct_PB6.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct_PB6.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_PB6.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_PB6);

  /* Configure limit switch negative x-axis (M1) GPIO using pin PB7 */
  GPIO_InitTypeDef GPIO_InitStruct_PC7;
  GPIO_InitStruct_PC7.Pin = GPIO_PIN_7;
  GPIO_InitStruct_PC7.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct_PC7.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_PC7.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_PC7);
}

/**
 * @brief Testing Limit Switch Function
 * @return //1 for successful, it would be stuck for unsuccessful (some sort of timeout then return 0?)
 * @param //Which motor to test
 */
uint8_t LimitSwitch_Test(uint8_t L6470_Id)
{
  // Run the motor forward
  L6470_Run(L6470_Id, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);

  // Tight poll until motor starts reverse direction (after limit switch is hit)
  while (L6470_CheckStatusRegisterFlag(L6470_Id, DIR_ID))
  {
  }

  // Run the motor backwards
  L6470_Run(L6470_Id, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);

  // Tight poll until the motor starts going in the forward direction (after limit switch is hit)
  while (!L6470_CheckStatusRegisterFlag(L6470_Id, DIR_ID))
  {
  }

  // If it reached here limit switches successfully ran
  return 1;
}

void ADC_Calculate_Offset()
{
  HAL_ADC_Start(&hadc1);
  uint32_t sum = 0;
  // Average 100 readings for better accuracy
  for (int i = 0; i < 100; i++)
  {
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    sum += HAL_ADC_GetValue(&hadc1);
  }
  // Store the offset value in global variable
  adc_offset = sum / 100;
}

void ADC_Calculate_Gain(float input_voltage)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
  uint32_t offset_corrected_value = raw_adc + adc_offset;

  float measured_voltage = (offset_corrected_value / 1023.0f) * 3.3f;
  adc_gain = input_voltage / measured_voltage;
}

/**
 * @}
 */
/* End of MicrosteppingMotor_Example */

/**
 * @}
 */
/* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
