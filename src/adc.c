/**
 ********************************************************************************
 * @file adc.c
 * @author Daniel Luchian, Jarryd Ross, Robson Lamond    
 * @brief Config and excution file for the ADC input from the potentiometer
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
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
uint32_t adc_offset = 0; //Calculated offset for the ADC
double adc_gain = 1.0;   //Calculated gain for the ADC

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
 * @brief Calculate ADC offset
 * @param None
 * @retval None
 */
void ADC1_Calculate_Offset()
{
  // Sum of 100 readings
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
 * @param input_voltage, the ideal voltage at the ADC
 * @retval None
 */
void ADC1_Calculate_Gain(float input_voltage)
{
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
  float offset_corrected_value = (float)raw_adc - adc_offset;

  float measured_voltage = (offset_corrected_value / 1023.0f) * 3.3f;
  adc_gain = input_voltage / measured_voltage;
}

/**
 * @brief Read ADC1 Value
 * @param None
 * @retval Calibrated ADC1 Value
 */
uint16_t ADC1_Calibrated_Read()
{
  // Poll ADC1 for a value
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);

  // Correct the reading
  float offset_corrected_value = (float)raw_adc - adc_offset;
  float offset_corrected_voltage = (offset_corrected_value / 1023.0f) * 3.3f;
  float corrected_voltage = offset_corrected_voltage * adc_gain;

  //Return the reading
  return (uint16_t)((corrected_voltage / 3.3f) * 1023);
}