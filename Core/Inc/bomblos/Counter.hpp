/*
 * Counter.hpp
 *
 *  Created on: Nov 27, 2018
 *      Author: michal
 */

#ifndef BOMBLOS_COUNTER_HPP_
#define BOMBLOS_COUNTER_HPP_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
//#include "stm32f4xx_hal_tim.h"
namespace bomblos{

template<typename ObjT >
class Counter{
	public:
		typedef void(ObjT::*CallbackT)();

		Counter(int period,CallbackT cb, ObjT* obj, TIM_HandleTypeDef &htim):
			_cb(cb),
			_obj(obj),
			_period(period),
			_state(0),
			_overflowed(0),
			_started(false){
			_htim=htim;
		};

		void resetState(){
			_state= 0;
		};

		void clearOverflow(){
			_overflowed = 0;
		}
		void inc(){
			if(!_started) return;
			_state++;
			if(_state >= +_period){
				_overflowed= 1;
				(_obj->*_cb)();
			}
		};

		uint32_t getState(){
			return _state;
		};

		void start(){
			HAL_TIM_Base_Start_IT(&_htim);
			_started = true;

		}

		void stop(){
			_started = false;
			HAL_TIM_Base_Stop_IT(&_htim);
		}

	private:
		uint8_t _started;
		uint32_t _state;
		uint32_t _period;
		TIM_HandleTypeDef _htim;
		uint8_t _overflowed;
		CallbackT _cb;
		ObjT* _obj;
};

}

#endif /* BOMBLOS_COUNTER_HPP_ */
