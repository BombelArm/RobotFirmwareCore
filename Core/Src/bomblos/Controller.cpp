#include <bomblos/Controller.hpp>
#include <bomblos/Motors.hpp>
#include <bomblos/encoders.h>

namespace bomblos{

Controller::Controller():
	counter(MSG_FREQ ,&Controller::desync_callback, this, htim2),
	state_pub(STATE_PUB_NAME, &state_msg),
	pos_sub(POS_SUB_NAME, &Controller::cmd_msg_callback, this),
	motors(),
	validator(0,0,0,0,0,0),
	previousPositions{0,0,0},
	isStopped(0)
{
	nh.initNode();
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	//publishers
	nh.advertise(state_pub);

	//subscribers
	nh.subscribe(pos_sub);
	previousTimestamp = 0;

	writeEncoderToAbsReg();

}

void Controller::publishState(){
	int16_t encoder0, encoder1, encoder2;

	encoder_read(&encoder0, 0);
	encoder_read(&encoder1, 1);
	encoder_read(&encoder2, 2);

	actualEncoders[0] = encoder0;
	actualEncoders[1] = encoder1;
	actualEncoders[2] = encoder2;

	state_msg.encoder0_pos = encoder0;
	state_msg.encoder1_pos = encoder1;
	state_msg.encoder2_pos = encoder2;

	state_msg.reg0_pos = actualAbsReg[0];
	state_msg.reg1_pos = actualAbsReg[1];
	state_msg.reg2_pos = actualAbsReg[2];

	state_pub.publish(&state_msg);
}

void Controller::cmd_msg_callback(const bombel_msgs::BombelCmd& cmd_msg){

	// ==== CMD DESCR =====
	// 0 - Stop
	// 1 - Start
	// 2 - setNextPosition
	// 3 - write pos from encoders to driver_reg
	// 4 - setPosition (driver ramp)
	// 5 - softStop


	int32_t nextAbsReg0 = motors.rad2AbsPosReg(0,cmd_msg.joint0_pos);
	int32_t nextAbsReg1= motors.rad2AbsPosReg(1,cmd_msg.joint1_pos);
	int32_t nextAbsReg2 = motors.rad2AbsPosReg(2,cmd_msg.joint2_pos);

	actualAbsReg[0] = motors.getRegPosition(0);
	actualAbsReg[1] = motors.getRegPosition(1);
	actualAbsReg[2] = motors.getRegPosition(2);

	BombelCmdType cmdType = static_cast<BombelCmdType>(cmd_msg.cmd);

	if(isStopped && cmdType != 2){
		return;

	}else if(cmdType == 0){
		isStopped = 1;

		motors.softStop(0);
		motors.softStop(1);
		motors.softStop(2);

		return;

	}else if(cmdType == 1){
		isStopped = 1;

		motors.hardStop(0);
		motors.hardStop(1);
		motors.hardStop(2);

		return;

	}else if(cmdType == 2){
		isStopped = 0;

		motors.setSpeed(0, 0);
		motors.setSpeed(1, 0);
		motors.setSpeed(2, 0);

		return;

	}else if(cmdType == 3){
		motors.setNextPosition(0, actualAbsReg[0], nextAbsReg0, MSG_FREQ);
		motors.setNextPosition(1, actualAbsReg[1], nextAbsReg1, MSG_FREQ);
		motors.setNextPosition(2, actualAbsReg[2], nextAbsReg2, MSG_FREQ);

		return;

	}else if(cmdType == 4){
		writeEncoderToAbsReg();

		return;

	}else if(cmdType == 5){
		motors.setPosition(0, nextAbsReg0);
		motors.setPosition(1, nextAbsReg1);
		motors.setPosition(2, nextAbsReg2);

		return;

	}
}

void Controller::writeEncoderToAbsReg(){
	int16_t encoder0, encoder1, encoder2;
	int32_t calcRegPos0, calcRegPos1, calcRegPos2;

	encoder0 = actualEncoders[0];
	encoder1 = actualEncoders[1];
	encoder2 = actualEncoders[2];

	calcRegPos0 = motors.encoder2AbsPosReg(0,encoder0);
	calcRegPos1 = motors.encoder2AbsPosReg(1,encoder1);
	calcRegPos2 = motors.encoder2AbsPosReg(2,encoder2);

	motors.setRegPosition(0, calcRegPos0);
	motors.setRegPosition(1, calcRegPos1);
	motors.setRegPosition(2, calcRegPos2);

	actualAbsReg[0] = calcRegPos0;
	actualAbsReg[1] = calcRegPos1;
	actualAbsReg[2] = calcRegPos2;

}

ros::NodeHandle& Controller::getNodeHandle(){
	return nh;
}

void Controller::desync_callback(){
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
}
Counter<Controller>& Controller::getCounter(){
	return counter;
}
}
