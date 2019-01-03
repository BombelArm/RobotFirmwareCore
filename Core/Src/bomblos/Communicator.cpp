#include <bomblos/Communicator.hpp>
#include <bomblos/MotorController.hpp>

Communicator::Communicator(MotorController *m1):
	counter(MSG_PERIOD ,&Communicator::desync_callback, this),
	status_pub(STATUS_PUB_NAME, &status_msg),
	speed_sub(SPEED_SUB_NAME, &Communicator::speed_msg_callback, this)
{
	nh.initNode();
//	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	//publishers
	nh.advertise(status_pub);

	//subscribers
	nh.subscribe(speed_sub);
	motor_controller = m1;
}

void Communicator::speed_msg_callback(const bombel_msgs::BombelSpeed& speed_msg){
	motor_controller->setSpeed(0,speed_msg.joint0_speed);
	motor_controller->setSpeed(1,speed_msg.joint1_speed);
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

