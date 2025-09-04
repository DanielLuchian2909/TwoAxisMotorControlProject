/**
 ********************************************************************************
 * @file pin_config.h 
 * @author Daniel Luchian, Jarryd Ross, Robson Lamond
 * @brief Config API for GPIO pins, ADC, and interrupts lines
 ********************************************************************************
 */

#ifndef __PIN_CONFIG_h__
#define __PIN_CONFIG_h__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/

/************************************
 * MACROS AND DEFINES
 ************************************/
#define LIMIT_SWITCH_XPOS_PORT GPIOA
#define LIMIT_SWITCH_XPOS_PIN GPIO_PIN_8

#define LIMIT_SWITCH_XNEG_PORT GPIOA
#define LIMIT_SWITCH_XNEG_PIN GPIO_PIN_9

#define LIMIT_SWITCH_YPOS_PORT GPIOB
#define LIMIT_SWITCH_YPOS_PIN GPIO_PIN_6

#define LIMIT_SWITCH_YNEG_PORT GPIOC
#define LIMIT_SWITCH_YNEG_PIN GPIO_PIN_7

#define AXIS_SWITCH_PORT GPIOB
#define AXIS_SWITCH_PIN GPIO_PIN_4

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void GPIO_LimitSwitch_Init(void);
void GPIO_AxisSwitch_Init(void);
void GPIO_ADC1_Init(void);

#ifdef __cplusplus
}
#endif

#endif // __PIN_CONFIG_H__