/*! 
 * \brief Encoders implementation
 * \addtogroup Encoders
 *  @{
 * \file
 * \brief Encoders implementation
 */

#ifndef ENCODER_LIB_H_
#define ENCODER_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include "limits.hpp"


#define HSPI hspi2
#define ENCODER_BITS 12

#define ENCODER0_OFFSET -341 /**<  \brief Encoder 0 offset - base position calibration */
#define ENCODER1_OFFSET -1288 /**<  \brief Encoder 1 offset - base position calibration */
#define ENCODER2_OFFSET -847/**<  \brief Encoder 2 offset - base position calibration */

#define ENCODER0_DIR	-1
#define ENCODER1_DIR	1
#define ENCODER2_DIR	1

/*!
 * \details This function reads angle from the encoder
 * \param data_in variable to which the angle will be written
 * \param cs encoder chip select (0,1,2)
 * \return HAL_StatusTypeDef  return code
 */


HAL_StatusTypeDef encoder_read(int16_t *data_in, int cs);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_ENCODER_H_ */
/** @}*/

