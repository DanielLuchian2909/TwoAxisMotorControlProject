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
#include "stdio.h"

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

/* Macros and Defines */
#define ADC1_RESOLUTION_BITS 1023

#define FAST_FWD_THRESHOLD_ADC       ADC1_RESOLUTION_BITS / 5
#define SLOW_FWD_THRESHOLD_ADC       ADC1_RESOLUTION_BITS / 5 * 2
#define STOP_THRESHOLD_ADC           ADC1_RESOLUTION_BITS / 5 * 3
#define SLOW_REV_THRESHOLD_ADC       ADC1_RESOLUTION_BITS / 5 * 4

//#define ENABLE_ADC_CALIBRATION //Comment to disable calibration

/* Global Variables */
//ADC Global Variables 
uint32_t adc_offset = 0;
double adc_gain = 1.0;

//Motor Running Variable
volatile eL6470_MotorRuning_t g_curr_motor;

/* Static Variables */

/* Global Function Prototypes */
uint16_t ADC_Calibrated_Read();
int __io_putchar(int ch);

/* Static Function Prototypes */
static void GPIO_LimitSwitch_Init(void);
static void GPIO_ADC_Init(void);
static void GPIO_AxisSwitch_Init(void);
static void ADC_Calculate_Offset();
static void ADC_Calculate_Gain(float input_voltage);

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

  /* Enable the GPIOA, B, C Clock*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Init the GPIO pins for the limit switch */
  GPIO_LimitSwitch_Init();

  /* Init the GPIO pin for motor switching */
  GPIO_AxisSwitch_Init();

  /* Init the GPIO pin for the ADC */
  GPIO_ADC_Init();

  /* Configure EXTI Line[9:5] and Line[4] interrupt priority */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);

  /* Enable EXTI Line[9:5] and Line[4] interrupt */
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* Intialize, Calibrate and Start ADC1 */
  MX_ADC1_Init();
  HAL_ADC_Start(&hadc1);
  #ifdef ENABLE_ADC_CALIBRATION
    ADC_Calculate_Offset();
    ADC_Calculate_Gain(3.3);
  #endif

  /* Initialize the motor control to start with Motor0*/
  //g_curr_motor = L6470_MOTOR0;

  /* Create variable to store and print ADC values*/
  char adc_buffer[20];
  uint16_t adc_value;

  L6470_Run(MOTOR_X, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);

  /* Infinite loop */
  while (1)
  {
    /* Check if any Application Command for L6470 has been entered by USART */
    // USART_CheckAppCmd();

    //sprintf(adc_buffer, "%d\r\n", adc_value);
    //USART_Transmit(&huart2, adc_buffer);
    //HAL_Delay(100);
    
    //Read the current ADC Value
    adc_value = ADC_Calibrated_Read();

    //If the ADC reading is between 0 and the FWD  drive FWD and fast
    if (adc_value < FAST_FWD_THRESHOLD_ADC)
    {
      L6470_Run(g_curr_motor, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    //If the ADC reading is between A and B bits drive FWD and slowly
    else if (adc_value < SLOW_FWD_THRESHOLD_ADC)
    {
      L6470_Run(g_curr_motor, L6470_DIR_FWD_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    //If the ADC reading is between B and C bits, stop
    else if (adc_value < STOP_THRESHOLD_ADC)
    {
      L6470_HardStop(g_curr_motor);
    }
    //If the ADC reading is between C and D bits drive REV and slowly
    else if (adc_value < SLOW_REV_THRESHOLD_ADC)
    {
      L6470_Run(g_curr_motor, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
    //If the ADC reading is greater than the REV fast speed threshold drive REV and slowly
    else
    {
      L6470_Run(g_curr_motor, L6470_DIR_REV_ID, L6470_SPEED_CONV * L6470_SLOW_SPEED);
    }
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
 * @brief Initiliaze the interrupt line for switching axis
 * @param None
 * @retval None
 */
static void GPIO_AxisSwitch_Init(void)
{
  /* Configure the interrupt pin for the motor direction */
  GPIO_InitTypeDef GPIO_InitStruct_Axis_Switch;
  GPIO_InitStruct_Axis_Switch.Pin = AXIS_SWITCH_PIN;
  GPIO_InitStruct_Axis_Switch.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_Axis_Switch.Pull = GPIO_PULLUP;
  GPIO_InitStruct_Axis_Switch.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AXIS_SWITCH_PORT, &GPIO_InitStruct_Axis_Switch);
}

/**
 * @brief Initiliaze the ADC
 * @param None
 * @retval None
 */
static void GPIO_ADC_Init(void)
{
  /* Configure Pin for ADC*/
  GPIO_InitTypeDef GPIO_InitStruct_ADC;
  GPIO_InitStruct_ADC.Pin = GPIO_PIN_4;
  GPIO_InitStruct_ADC.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct_ADC.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_ADC);
}

/**
 * @brief Calculate ADC offset
 * @param None
 * @retval None
 */
static void ADC_Calculate_Offset()
{
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

/**
 * @brief Calculate ADC gain
 * @param None
 * @retval None
 */
static void ADC_Calculate_Gain(float input_voltage)
{
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
  float offset_corrected_value = (float)raw_adc - adc_offset;

  float measured_voltage = (offset_corrected_value / 1023.0f) * 3.3f;
  adc_gain = input_voltage / measured_voltage;
}

/**
 * @brief Read ADC Value
 * @param None
 * @retval ADC Value
 */
uint16_t ADC_Calibrated_Read()
{
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
  float offset_corrected_value = (float)raw_adc - adc_offset;
  float offset_corrected_voltage = (offset_corrected_value / 1023.0f) * 3.3f;
  float corrected_voltage = offset_corrected_voltage * adc_gain;
  return (uint16_t)((corrected_voltage / 3.3f) * 1023);
}

/**
 * @brief Putchar Function
 * @param Character to transmit
 * @retval Transmitted character
 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
