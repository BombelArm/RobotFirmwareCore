#include <bomblos/Controller.hpp>
#include <bomblos/Motors.hpp>
#include <bomblos/encoders.h>

namespace bomblos{

Controller::Controller():
	counter(MSG_PERIOD ,&Controller::desync_callback, this, htim2),
	state_pub(STATE_PUB_NAME, &state_msg),
	pos_sub(POS_SUB_NAME, &Controller::pos_msg_callback, this),
	motors(MSG_PERIOD),
	validator(0,0,0,0,0,0)
{
	nh.initNode();
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	//publishers
	nh.advertise(state_pub);

	//subscribers
	nh.subscribe(pos_sub);
}

void Controller::publishState(){
	int16_t encoder0, encoder1, encoder2;

	encoder_read(&encoder0,0);
	encoder_read(&encoder1,1);
	encoder_read(&encoder2,2);

//	encoder0 = motors.getRegPosition(0);
//	encoder1 = motors.getRegPosition(1);
//	encoder2 = motors.getRegPosition(2);
//
	state_msg.encoder0_pos = encoder0;
	state_msg.encoder1_pos = encoder1;
	state_msg.encoder2_pos = encoder2;

//	state_msg.encoder0_pos = motors.getRegPosition(0);
//	state_msg.encoder1_pos = motors.getRegPosition(1);
//	state_msg.encoder2_pos = motors.getRegPosition(2);

	state_pub.publish(&state_msg);
}

void Controller::pos_msg_callback(const bombel_msgs::BombelPos& pos_msg){
	int32_t motor0AbsReg = motors.rad2AbsPosReg(0,pos_msg.joint0_pos);
	int32_t motor1AbsReg = motors.rad2AbsPosReg(1,pos_msg.joint1_pos);
	int32_t motor2AbsReg = motors.rad2AbsPosReg(2,pos_msg.joint2_pos);

//	motors.setPosition(0, motor0AbsReg);
//	motors.setPosition(1, motor1AbsReg);
//	motors.setPosition(2, motor2AbsReg);
	motors.setSpeed(2,motor2AbsReg);
}

ros::NodeHandle& Controller::getNodeHandle(){
	return nh;
}

Counter<Controller>& Controller::getCounter(){
	return counter;
}

void Controller::desync_callback(){
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
}

}
