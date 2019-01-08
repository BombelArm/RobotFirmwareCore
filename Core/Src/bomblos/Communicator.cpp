#include <bomblos/Communicator.hpp>
#include <bomblos/MotorController.hpp>
#include <bomblos/encoders.h>

namespace bomblos{

Communicator::Communicator(MotorController *m1):
	counter(MSG_PERIOD ,&Communicator::desync_callback, this, htim2),
	state_pub(STATE_PUB_NAME, &state_msg)
//	speed_sub(SPEED_SUB_NAME, &Communicator::speed_msg_callback, this)
{
	nh.initNode();
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	//publishers
	nh.advertise(state_pub);

	//subscribers
//	nh.subscribe(speed_sub);
	motor_controller = m1;
}

void Communicator::publishState(){
	int16_t encoder0, encoder1, encoder2;

	encoder_read(&encoder0,0);
	encoder_read(&encoder1,1);
	encoder_read(&encoder2,2);
	state_msg.encoder0_pos = encoder0;
	state_msg.encoder1_pos = encoder1;
	state_msg.encoder2_pos = encoder2;

	state_pub.publish(&state_msg);
}

void Communicator::speed_msg_callback(const bombel_msgs::BombelSpeed& speed_msg){
//	motor_controller->setSpeed(0,speed_msg.joint0_speed);
//	motor_controller->setSpeed(1,speed_msg.joint1_speed);
//	motor_controller->setSpeed(2,speed_msg.joint2_speed);
//	motor_controller->setSpeed(3,speed_msg.joint3_speed);
}

ros::NodeHandle& Communicator::getNodeHandle(){
	return nh;
}

Counter<Communicator>& Communicator::getCounter(){
	return counter;
}

void Communicator::desync_callback(){
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
}

}
