/*
 * PID.hpp
 *
 *  Created on: Jan 17, 2019
 *      Author: osboxes
 */

#ifndef INC_BOMBLOS_PID_HPP_
#define INC_BOMBLOS_PID_HPP_

#include "stdint.h"

class PID{
private:
	int32_t dt;
	int32_t u_max;
	int32_t u_min;
	float Kp;
	float Kd;
	float Ki;
	int32_t e_k_2; //e(k-2)
	int32_t e_k_1; //e(k-1)
	int32_t u_k_1; //u(k-1)

public:
	PID(int32_t u_max, int32_t u_min, float Kp, float Kd, float Ki);
	int32_t calc(int32_t pv, int32_t setpoint, int32_t Tmillis);
};


#endif /* INC_BOMBLOS_PID_HPP_ */
