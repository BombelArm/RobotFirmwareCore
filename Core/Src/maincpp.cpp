/*
 * maincpp.cpp
 *
 *  Created on: 04.07.2018
 *      Author: mromanow
 */

#include "maincpp.h"
#include "xnucleoihm02a1.h"
#include "motors_param.h"
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <bomblos/MotorController.hpp>
#include <bomblos/Communicator.hpp>


extern TIM_HandleTypeDef htim2;

MotorController *motors;
Communicator *com;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	com->getNodeHandle().getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	com->getNodeHandle().getHardware()->reset_rbuf();
}


void setup(void)
{
//	init_motors();
//	set_speed(0,100);
	motors = new MotorController();
	com = new Communicator(motors);

}

void loop(void)
{
	com->getNodeHandle().spinOnce();
}
