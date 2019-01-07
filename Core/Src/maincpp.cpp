/*
 * maincpp.cpp
 *
 *  Created on: 04.07.2018
 *      Author: mromanow
 */

#include "maincpp.h"
#include "xnucleoihm02a1.h"
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <bomblos/MotorController.hpp>
#include <bomblos/Communicator.hpp>


bomblos::MotorController *motors;
bomblos::Communicator *com;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(com) com->getNodeHandle().getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(com) com->getNodeHandle().getHardware()->reset_rbuf();
}


void setup(void)
{
	motors = new bomblos::MotorController();
	com = new bomblos::Communicator(motors);
}

void loop(void)
{
	com->getNodeHandle().spinOnce();
}
