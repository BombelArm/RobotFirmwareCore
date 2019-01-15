/*
 * communicator.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: michal
 */

#ifndef BOMBLOS_CONTROLLER_HPP_
#define BOMBLOS_CONTROLLER_HPP_

#include <bomblos/Counter.hpp>
#include <bomblos/Motors.hpp>
#include <bomblos/Validator.hpp>
#include <bombel_msgs/BombelPos.h>
#include <bombel_msgs/BombelState.h>
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int16.h>

#define POS_SUB_NAME "bombel/pos"
#define STATE_PUB_NAME "bombel/state"
#define MSG_PERIOD 100 //in 10ms == 100Hz


extern TIM_HandleTypeDef htim2;

namespace bomblos{

class Controller
{
	private:

		//ROS
		ros::NodeHandle 			nh;
		bombel_msgs::BombelState	state_msg;
		ros::Publisher				state_pub;
		ros::Subscriber				<bombel_msgs::BombelPos, Controller> 	pos_sub;

		//bomblos
		Validator					validator;
		Motors 						motors;


		void pos_msg_callback(const bombel_msgs::BombelPos& pos_msg);

		//SYNC
		Counter<Controller> counter; //counts in ms (TIM3)
		void desync_callback();

	public:
		Controller();

		void publishState();

		ros::NodeHandle& 	getNodeHandle();
		Counter<Controller>&	getCounter();
};

}



#endif /* BOMBLOS_CONTROLLER_HPP_ */
