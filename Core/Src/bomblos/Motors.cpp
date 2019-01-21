/*
 * MotorController.cpp
 *
 *  Created on: Dec 18, 2018
 *      Author: michal
 */

#include "bomblos/Motors.hpp"
#include "bomblos/encoders.h"
#include <cmath>

namespace bomblos{

Motors::Motors():
	MotorParameterInitData	{
	  {
		{
				MOTOR_SUPPLY_VOLTAGE,
				MOTOR_0_MIN_STEPS_PER_REVOLUTION,
				MAX_MOTOR_PHASE_VOLTAGE_AMPR,
				MAX_MOTOR_PHASE_VOLTAGE_VOLT,
				MOTOR_0_INIT_SPEED,
				MOTOR_0_ACC,
				MOTOR_0_DECC,
				MOTOR_0_MAX_SPEED,
				MOTOR_MIN_SPEED,
				MOTOR_SPEED_THRESHOLD,
				MOTOR_HOLDING_KVAL,
				MOTOR_CONST_SPEED_KVAL,
				MOTOR_ACC_START_KVAL,
				MOTOR_DECC_START_KVAL,
				MOTOR_INTERSECT_SPEED,
				MOTOR_START_SLOPE,
				MOTOR_ACC_FINAL_SLOPE,
				MOTOR_DECC_FINAL_SLOPE,
				MOTOR_THERMAL_COMP,
				MOTOR_OCD_THRESHOLD,
				MOTOR_STALL_THRESHOLD,
				MOTOR_MICROSTEP,
				MOTOR_ALARM_CONDITIOnS,
				MOTOR_IC_CONFIG
		},
		{
				MOTOR_SUPPLY_VOLTAGE,
				MOTOR_1_MIN_STEPS_PER_REVOLUTION,
				MAX_MOTOR_PHASE_VOLTAGE_AMPR,
				MAX_MOTOR_PHASE_VOLTAGE_VOLT,
				MOTOR_1_INIT_SPEED,
				MOTOR_1_ACC,
				MOTOR_1_DECC,
				MOTOR_1_MAX_SPEED,
				MOTOR_MIN_SPEED,
				MOTOR_SPEED_THRESHOLD,
				MOTOR_HOLDING_KVAL,
				MOTOR_CONST_SPEED_KVAL,
				MOTOR_ACC_START_KVAL,
				MOTOR_DECC_START_KVAL,
				MOTOR_INTERSECT_SPEED,
				MOTOR_START_SLOPE,
				MOTOR_ACC_FINAL_SLOPE,
				MOTOR_DECC_FINAL_SLOPE,
				MOTOR_THERMAL_COMP,
				MOTOR_OCD_THRESHOLD,
				MOTOR_STALL_THRESHOLD,
				MOTOR_MICROSTEP,
				MOTOR_ALARM_CONDITIOnS,
				MOTOR_IC_CONFIG
		}
	  },
	  {
		{
				MOTOR_SUPPLY_VOLTAGE,
				MOTOR_2_MIN_STEPS_PER_REVOLUTION,
				MAX_MOTOR_PHASE_VOLTAGE_AMPR,
				MAX_MOTOR_PHASE_VOLTAGE_VOLT,
				MOTOR_2_INIT_SPEED,
				MOTOR_2_ACC,
				MOTOR_2_DECC,
				MOTOR_2_MAX_SPEED,
				MOTOR_MIN_SPEED,
				MOTOR_SPEED_THRESHOLD,
				MOTOR_HOLDING_KVAL,
				MOTOR_CONST_SPEED_KVAL,
				MOTOR_ACC_START_KVAL,
				MOTOR_DECC_START_KVAL,
				MOTOR_INTERSECT_SPEED,
				MOTOR_START_SLOPE,
				MOTOR_ACC_FINAL_SLOPE,
				MOTOR_DECC_FINAL_SLOPE,
				MOTOR_THERMAL_COMP,
				MOTOR_OCD_THRESHOLD,
				MOTOR_STALL_THRESHOLD,
				MOTOR_MICROSTEP,
				MOTOR_ALARM_CONDITIOnS,
				MOTOR_IC_CONFIG
		},
		{
				MOTOR_SUPPLY_VOLTAGE,
				MOTOR_2_MIN_STEPS_PER_REVOLUTION,
				MAX_MOTOR_PHASE_VOLTAGE_AMPR,
				MAX_MOTOR_PHASE_VOLTAGE_VOLT,
				MOTOR_2_INIT_SPEED,
				MOTOR_2_ACC,
				MOTOR_2_DECC,
				MOTOR_2_MAX_SPEED,
				MOTOR_MIN_SPEED,
				MOTOR_SPEED_THRESHOLD,
				MOTOR_HOLDING_KVAL,
				MOTOR_CONST_SPEED_KVAL,
				MOTOR_ACC_START_KVAL,
				MOTOR_DECC_START_KVAL,
				MOTOR_INTERSECT_SPEED,
				MOTOR_START_SLOPE,
				MOTOR_ACC_FINAL_SLOPE,
				MOTOR_DECC_FINAL_SLOPE,
				MOTOR_THERMAL_COMP,
				MOTOR_OCD_THRESHOLD,
				MOTOR_STALL_THRESHOLD,
				MOTOR_MICROSTEP,
				MOTOR_ALARM_CONDITIOnS,
				MOTOR_IC_CONFIG
		}
	  }
	},
	motorNativeDirections{
		MOTOR_0_FDIR,
		MOTOR_1_FDIR,
		MOTOR_2_FDIR,
	},
	motorMicroStepMultiplier{
		0,
		0,
		0
	},
	motorStepsPerJointRev{
		0,
		0,
		0
	},
	motorPrevAbsPos{
		0,
		0,
		0
	}
{

	for(int i=0; i<JOINTS_N;i++){
		uint8_t row, col;

		if(i<=1){
			row=i;
			col=0;
		}else if(i<=3){
			row=i-2;
			col=1;
		}

		switch(MotorParameterInitData[col][row].step_sel){
			case FULL_STEP:
				motorMicroStepMultiplier[i]=1;
				break;
			case MICROSTEP_1_4:
				motorMicroStepMultiplier[i]=4;
				break;
			case MICROSTEP_1_8:
				motorMicroStepMultiplier[i]=8;
				break;
			case MICROSTEP_1_16:
				motorMicroStepMultiplier[i]=16;
				break;
			case MICROSTEP_1_32:
				motorMicroStepMultiplier[i]=32;
				break;
			case MICROSTEP_1_64:
				motorMicroStepMultiplier[i]=64;
				break;
			case MICROSTEP_1_128:
				motorMicroStepMultiplier[i]=128;
				break;
		}

		motorStepsPerJointRev[i]=MotorParameterInitData[col][row].fullstepsperrevolution;
		switch(i){
			case 0:
				motorStepsPerJointRev[i]*=JOINT_0_SHIFT;
				break;
			case 1:
				motorStepsPerJointRev[i]*=JOINT_1_SHIFT;
				break;
			case 2:
				motorStepsPerJointRev[i]*=JOINT_2_SHIFT;
				break;
		}

	}

	initMotors();
}

int32_t Motors::absPosReg2Encoder(uint8_t motor, int32_t absPos){
	int32_t encoderPos;
	uint16_t microStepMultiplier = motorMicroStepMultiplier[motor];
	float fullStepPerRev = motorStepsPerJointRev[motor];

    encoderPos = pow(2,ENCODER_BITS)*absPos/(fullStepPerRev*microStepMultiplier);

	return encoderPos;
}

int32_t Motors::encoder2AbsPosReg(uint8_t motor, int32_t absPos){
	return 1;
}

int32_t Motors::rad2AbsPosReg(uint8_t motor, float rad){
	int32_t absPosReg;
	uint16_t microStepMultiplier = motorMicroStepMultiplier[motor];
	float fullStepPerRev = motorStepsPerJointRev[motor];

	absPosReg = rad*fullStepPerRev*microStepMultiplier/(2*M_PI);

	return absPosReg;
}

void Motors::initMotors(){
	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;


	for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
	{
		StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
		MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+(id*L6470DAISYCHAINSIZE);
		StepperMotorBoardHandle->Config(MotorParameterDataSingle);
	}
}

void Motors::setSpeed(uint8_t motor, uint32_t speed){
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	MotorParameterData_t *MotorParameterDataSingle;
	uint8_t board, device;
	uint32_t Step, i, _speed;
	eL6470_DirId_t dir;

	dir = motorNativeDirections[motor];

	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}

	if(speed < 0){
		speed= -speed;
		//changing direction
		if(dir == L6470_DIR_FWD_ID) dir = L6470_DIR_REV_ID;
		else dir = L6470_DIR_FWD_ID;
	}

    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);
    MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);
	_speed = Step_s_2_Speed(speed);
	StepperMotorBoardHandle->Command->Run(board,device, dir, _speed);

}

void Motors::setPosition(uint8_t motor, int32_t position){
	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	uint8_t board, device;


	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}


	StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);
	if(motorNativeDirections[motor] == L6470_DIR_REV_ID) position = -position;
	uint32_t _position = Position_2_AbsPos(position);
	MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);
	StepperMotorBoardHandle->Command->GoTo(board,device, _position);
}

void Motors::move(uint8_t motor, uint32_t steps){
	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	uint8_t board, device;


	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}

	StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);
	MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);
	StepperMotorBoardHandle->Command->Move(board,device,L6470_DIR_FWD_ID,steps);

}

void Motors::setNextPosition(uint8_t motor, int32_t position, uint16_t time){
	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	uint8_t board, device;
	eL6470_DirId_t dir;
	dir = motorNativeDirections[motor];


	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}

	StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

	int32_t distance = position - getRegPosition(motor);
	if(distance< 0){
		distance= -distance;
		//changing direction
		if(dir == L6470_DIR_FWD_ID) dir = L6470_DIR_REV_ID;
		else dir = L6470_DIR_FWD_ID;
	}

	float full_steps_s=(float) distance*time/motorMicroStepMultiplier[motor];
	uint32_t speed = Step_s_2_Speed(full_steps_s);
	MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);
	StepperMotorBoardHandle->Command->Run(board,device,dir,speed);
}

int32_t  Motors::getRegPosition(uint8_t motor){

	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	uint8_t board, device;
	uint32_t _speed;


	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}

    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

	MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);
	uint32_t regVal= StepperMotorBoardHandle->Command->GetParam(board,device,L6470_ABS_POS_ID);

	int32_t abs_pos = AbsPos_2_Position(regVal);
	if(motorNativeDirections[motor] == L6470_DIR_REV_ID) abs_pos = -abs_pos;

	return abs_pos;

}

int16_t  Motors::getEncoderPosition(uint8_t motor){
	int16_t pos;
	encoder_read(&pos,motor);

	return pos;
}

sL6470_StatusRegister_t Motors::getStatus(uint8_t motor){
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	union{
		uint16_t resp;
		sL6470_StatusRegister_t reg;
	}parser;
	uint8_t board, device;

	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

	parser.resp=(uint16_t) StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->GetParam(device,L6470_STATUS_ID);

	return parser.reg;

}

void Motors::softStop(uint8_t motor){
	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	uint8_t board, device;
	uint32_t _speed;

	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

	MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);

	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftHiZ(device);
	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

}

void Motors::hardStop(uint8_t motor){
	MotorParameterData_t *MotorParameterDataSingle;
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
	uint8_t board, device;
	uint32_t _speed;

	if(motor == 0 || motor == 1){
		board = EXPBRD_ID(0);
		device = L6470_ID(motor);
	}else if(motor == 2 || motor == 3){
		board = EXPBRD_ID(1);
		device = L6470_ID(motor - 2);
	}
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

	MotorParameterDataSingle = (MotorParameterData_t*)MotorParameterInitData+((board*L6470DAISYCHAINSIZE)+device);

	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

}

}
