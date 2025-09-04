/**
 ********************************************************************************
 * @file gpio_config.c
 * @author Daniel Luchian, Jarryd Ross, Robson Lamond
 * @brief GPIO config file 
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "gpio_config.h"
#include "xnucleoihm02a1_interface.h"

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
/**
 * @brief Initiliaze the gpio pins for interrupt lines for the limit switches
 * @param None
 * @retval None
 */
void GPIO_LimitSwitch_Init(void)
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
 * @brief Initiliaze the GPIO pin for the interrupt line for switching axis
 * @param None
 * @retval None
 */
void GPIO_AxisSwitch_Init(void)
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
void GPIO_ADC1_Init(void)
{
  /* Configure Pin for ADC*/
  GPIO_InitTypeDef GPIO_InitStruct_ADC;
  GPIO_InitStruct_ADC.Pin = GPIO_PIN_0;
  GPIO_InitStruct_ADC.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct_ADC.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_ADC);
}