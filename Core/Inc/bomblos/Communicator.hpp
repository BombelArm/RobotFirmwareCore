/*
 * communicator.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: michal
 */

#ifndef BOMBLOS_COMMUNICATOR_HPP_
#define BOMBLOS_COMMUNICATOR_HPP_

#include <bomblos/Counter.hpp>
#include <bomblos/MotorController.hpp>
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <bombel_msgs/BombelSpeed.h>
#include <std_msgs/UInt32.h>

#define SPEED_SUB_NAME "bombel/speed"
#define STATUS_PUB_NAME "bombel/status"
#define MSG_PERIOD 10 //in 10ms == 100Hz


extern TIM_HandleTypeDef htim2;

namespace bomblos{

class Communicator
{
	private:

		//ROS
		ros::NodeHandle 	nh;
		std_msgs::UInt64  	status_msg;
		ros::Publisher		status_pub;
		ros::Subscriber		<bombel_msgs::BombelSpeed, Communicator> 	speed_sub;
		MotorController 	*motor_controller;

		void speed_msg_callback(const bombel_msgs::BombelSpeed& speed_msg);

		//SYNC
		Counter<Communicator> counter; //counts in ms (TIM3)
		void desync_callback();

	public:
		Communicator(MotorController *m1);
		ros::NodeHandle& 	getNodeHandle();
		Counter<Communicator>&	getCounter();
};

}



#endif /* BOMBLOS_COMMUNICATOR_HPP_ */
