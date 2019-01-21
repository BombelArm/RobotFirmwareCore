/*
 * PID.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: osboxes
 */


#include "bomblos/PID.hpp"


PID::PID(int32_t u_max, int32_t u_min, float Kp, float Kd, float Ki){
	PID::u_max = u_max;
	PID::u_min = u_min;
	PID::Kp = Kp;
	PID::Kd = Kd;
	PID::Ki = Ki;
	PID::dt = 0;
	e_k_2 = 0;
	e_k_1 = 0;
	u_k_1 = 0;
}

int32_t PID::calc(int32_t pv, int32_t setpoint, int32_t T_millis){

	float r2 = (Kp*Kd)/T_millis;
	float r1 = Kp * ( T_millis/(2*Ki) - 2*(Kd/T_millis) - 1);
	float r0 = Kp * ( 1 + T_millis/(2*Ki) + Kd/T_millis);

	int32_t e = setpoint - pv;

	int32_t u = r2*e_k_2 + r1*e_k_1 + r0*e + u_k_1;

	e_k_2 = e_k_1;
	e_k_1 = e;
	u_k_1 = u;

	if(u>u_max) u = u_max;
	if(u<u_min) u = u_min;

	return u;
}
