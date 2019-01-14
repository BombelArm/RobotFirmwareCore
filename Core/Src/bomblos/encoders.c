/*! \file
 * \brief Encoders source code.
 */


#ifndef ENCODER_LIB_C_
#define ENCODER_LIB_C_

#include <bomblos/encoders.h>
#include "spi.h"
#include <math.h>


HAL_StatusTypeDef encoder_read(int16_t *data_in,int cs)
{
	HAL_StatusTypeDef status;
	GPIO_TypeDef*     port;
	uint16_t	  pin;
	uint16_t	  data;

	int8_t		  e_dirs[JOINTS_N]={
							ENCODER0_DIR,
							ENCODER1_DIR,
							ENCODER2_DIR
					};
	GPIO_TypeDef* e_ports[JOINTS_N]={
							ENCODER_CS0_GPIO_Port,
							ENCODER_CS1_GPIO_Port,
							ENCODER_CS2_GPIO_Port
					};
	uint16_t	  e_pins[JOINTS_N]={
							ENCODER_CS0_Pin,
							ENCODER_CS1_Pin,
							ENCODER_CS2_Pin
					};
	int16_t	  e_offests[JOINTS_N]={
							ENCODER0_OFFSET,
							ENCODER1_OFFSET,
							ENCODER2_OFFSET
						};


	port=e_ports[cs];
	pin=e_pins[cs];

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	status=HAL_SPI_Receive(&HSPI   , &data, 1, 0x00ff);
	data=(uint16_t) data>>4;
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

	data+=e_offests[cs];
	data&=0x0FFF;

	if(data <2048 && data>=0){
			(*data_in)=data*e_dirs[cs];
	}else{
			(*data_in)=-fabs(data-4096)*e_dirs[cs];
	}


	return status;
}



#endif /* ENCODER_LIB_H_ */
