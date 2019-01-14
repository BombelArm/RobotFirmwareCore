/*
 * MotorController.hpp
 *
 *  Created on: Dec 18, 2018
 *      Author: michal
 */

#ifndef BOMBLOS_MOTORS_HPP_
#define BOMBLOS_MOTORS_HPP_

#include "stdint.h"

#include "bomblos/limits.hpp"

#include "L6470.h"
#include "xnucleoihm02a1.h"
#include "stm32f4xx_hal.h"

//Constants for all motors
#define MOTOR_SUPPLY_VOLTAGE 			12.0				//!< motor supply voltage in V
#define MAX_MOTOR_PHASE_VOLTAGE_AMPR 	1.0					//!< max motor phase voltage in A
#define MAX_MOTOR_PHASE_VOLTAGE_VOLT 	3.06				//!< max motor phase voltage in V
#define MOTOR_INIT_SPEED 				300.0				//!< motor initial speed [step/s]
#define MOTOR_ACC 						500.0				//!< motor acceleration [step/s^2] (comment for infinite acceleration mode)
#define MOTOR_DECC 						500.0				//!< motor deceleration [step/s^2] (comment for infinite deceleration mode)
#define MOTOR_MAX_SPEED 				2000.0				//!< motor maximum speed [step/s]
#define MOTOR_MIN_SPEED 				0.0					//!< motor minimum speed [step/s]
#define MOTOR_SPEED_THRESHOLD 			602.7				//!< motor full-step speed threshold [step/s]
#define MOTOR_HOLDING_KVAL 				3.06				//!< holding kval [V]
#define MOTOR_CONST_SPEED_KVAL 			3.06				//!< constant speed kval [V]
#define MOTOR_ACC_START_KVAL 			3.06				//!< acceleration starting kval [V]
#define MOTOR_DECC_START_KVAL 			3.06				//!< deceleration starting kval [V]
#define MOTOR_INTERSECT_SPEED 			61.52				//!< intersect speed for bemf compensation curve slope changing [step/s]
#define MOTOR_START_SLOPE 				392.1569e-6			//!< start slope [s/step]
#define MOTOR_ACC_FINAL_SLOPE 			643.1372e-6			//!< acceleration final slope [s/step]
#define MOTOR_DECC_FINAL_SLOPE 			643.1372e-6			//!< deceleration final slope [s/step]
#define MOTOR_THERMAL_COMP 				0					//!< thermal compensation factor (range [0, 15])
#define MOTOR_OCD_THRESHOLD 			3.06*1000*1.10		//!< ocd threshold [ma] (range [375 ma, 6000 ma])
#define MOTOR_STALL_THRESHOLD 			3.06*1000*1.00		//!< stall threshold [ma] (range [31.25 ma, 4000 ma])
#define MOTOR_MICROSTEP 				MICROSTEP_1_128		//!< step mode selection
//#define MOTOR_MICROSTEP 				FULL_STEP		//!< step mode selection
#define MOTOR_ALARM_CONDITIOnS 			0xFF				//!< alarm conditions enable
#define MOTOR_IC_CONFIG 				0x2E88				//!< ic configuration


//Constants for individual motors
#define MOTOR_0_FDIR L6470_DIR_FWD_ID
#define MOTOR_1_FDIR L6470_DIR_FWD_ID
#define MOTOR_2_FDIR L6470_DIR_REV_ID
#define MOTOR_0_MIN_STEPS_PER_REVOLUTION 		200					//!< min number of steps per revolution for the motor
#define MOTOR_1_MIN_STEPS_PER_REVOLUTION 		200 * 5.18
#define MOTOR_2_MIN_STEPS_PER_REVOLUTION 		200
#define JOINT_0_SHIFT 8.75
#define JOINT_1_SHIFT 3.3333
#define JOINT_2_SHIFT 3.8

//#define JOINT_0_

namespace bomblos{

class Motors{
private:
	MotorParameterData_t MotorParameterInitData[EXPBRD_MOUNTED_NR][L6470DAISYCHAINSIZE];
	eL6470_DirId_t motorNativeDirections[JOINTS_N];
	uint16_t motorMicroStepMultiplier[JOINTS_N];
	float motorStepsPerJointRev[JOINTS_N];
	int32_t motorPrevAbsPos[JOINTS_N];

	uint16_t nextStepPeriod;


	void initMotors();


public:
	Motors(uint16_t nextStepPeriod);

	void setSpeed(uint8_t motor, float speed);
	void setPosition(uint8_t motor, int32_t position);
	void setNextPosition(uint8_t motor, int32_t position);
	void softStop(uint8_t motor);
	void hardStop(uint8_t motor);

	int32_t  getRegPosition(uint8_t motor);
	int16_t  getEncoderPosition(uint8_t motor);
	sL6470_StatusRegister_t getStatus(uint8_t motor);

	int32_t absPosReg2Encoder(uint8_t motor, int32_t absPos);
	int32_t encoder2AbsPosReg(uint8_t motor, int32_t absPos);
	int32_t rad2AbsPosReg(uint8_t motor, float rad);
};

}

#endif /* BOMBLOS_MOTORS_HPP_ */
