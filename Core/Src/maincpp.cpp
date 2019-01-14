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
#include <bomblos/Controller.hpp>


bomblos::Controller *controller;
ros::NodeHandle *nh;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(nh) nh->getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(nh) nh->getHardware()->reset_rbuf();
}
void setup(void)
{
	controller = new bomblos::Controller();
	nh = &controller->getNodeHandle();
}

void loop(void)
{
	nh->spinOnce();
	controller->publishState();
	HAL_Delay(100);
}
