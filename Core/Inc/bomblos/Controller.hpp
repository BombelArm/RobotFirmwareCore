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
#include <bomblos/limits.hpp>
#include <bomblos/Validator.hpp>
#include <bombel_msgs/BombelCmd.h>
#include <bombel_msgs/BombelState.h>
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int16.h>

#define POS_SUB_NAME "bombel/cmd"
#define STATE_PUB_NAME "bombel/state"
#define MSG_FREQ 20 //20Hz
#define MAX_DRIVER_ENCODER_DIFF_RAD 0.07 //max difference between drivers' registers and encoders' readings

extern TIM_HandleTypeDef htim2;

namespace bomblos{

enum BombelCmdType{
	SoftStop = 0,
	HardStop,
	Start,
	SetNextPosition,
	WriteEncodersToDriver,
	SetPosition
};

class Controller
{
	private:

		//ROS
		ros::NodeHandle 			nh;
		bombel_msgs::BombelState	state_msg;
		ros::Publisher				state_pub;
		ros::Subscriber				<bombel_msgs::BombelCmd, Controller> 	pos_sub;

		//bomblos
		Validator					validator;
		Motors 						motors;

		int32_t previousPositions[JOINTS_N];
		uint32_t previousTimestamp;

		void cmd_msg_callback(const bombel_msgs::BombelCmd& cmd_msg);

		//SYNC
		Counter<Controller> counter; //counts in ms (TIM3)
		void desync_callback();

	public:
		Controller();

		void publishState();
		void writeEncoderToAbsReg();

		ros::NodeHandle& 	getNodeHandle();
		Counter<Controller>&	getCounter();

		int32_t actualAbsReg[JOINTS_N];
		int32_t actualEncoders[JOINTS_N];

		int16_t isStopped;
		const int16_t maxDriverEncoderDiff; //in bits
};

}



#endif /* BOMBLOS_CONTROLLER_HPP_ */
