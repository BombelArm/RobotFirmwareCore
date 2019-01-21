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


extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

bomblos::Controller *controller;
ros::NodeHandle *nh;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(nh) nh->getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(nh) nh->getHardware()->reset_rbuf();
}

void TIM3_PeriodElapsedCallback(){}
void TIM4_PeriodElapsedCallback(){
	if(nh) nh->spinOnce();
}
void TIM5_PeriodElapsedCallback(){
	if(controller) controller->publishState();
}

void setup(void)
{
	controller = new bomblos::Controller();
	nh = &controller->getNodeHandle();

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
}

void loop(void)
{
}
