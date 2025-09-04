/**
 ********************************************************************************
 * @file adc.h 
 * @author Daniel Luchian, Jarryd Ross, Robson Lamond
 * @brief Header file and API for ADC1 (potientometer input)
 ********************************************************************************
 */

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include <stdint.h>

/************************************
 * MACROS AND DEFINES
 ************************************/
#define ADC1_RESOLUTION_BITS 1023

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void ADC1_Calculate_Offset();  //Calculates the offset of ADC1
void ADC1_Calculate_Gain(float input_voltage); //Calculates gain of ADC1
uint16_t ADC1_Calibrated_Read(void); // Reads calibrated ADC Val

#ifdef __cplusplus
}
#endif

#endif // __ADC_H__