#ifndef _ROS_bombel_msgs_BombelState_h
#define _ROS_bombel_msgs_BombelState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bombel_msgs
{

  class BombelState : public ros::Msg
  {
    public:
      typedef int16_t _encoder0_pos_type;
      _encoder0_pos_type encoder0_pos;
      typedef int16_t _encoder1_pos_type;
      _encoder1_pos_type encoder1_pos;
      typedef int16_t _encoder2_pos_type;
      _encoder2_pos_type encoder2_pos;
      typedef int32_t _reg0_pos_type;
      _reg0_pos_type reg0_pos;
      typedef int32_t _reg1_pos_type;
      _reg1_pos_type reg1_pos;
      typedef int32_t _reg2_pos_type;
      _reg2_pos_type reg2_pos;
      typedef uint16_t _driverPositionError_type;
      _driverPositionError_type driverPositionError;
      typedef bool _isStopped_type;
      _isStopped_type isStopped;

    BombelState():
      encoder0_pos(0),
      encoder1_pos(0),
      encoder2_pos(0),
      reg0_pos(0),
      reg1_pos(0),
      reg2_pos(0),
      driverPositionError(0),
      isStopped(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_encoder0_pos;
      u_encoder0_pos.real = this->encoder0_pos;
      *(outbuffer + offset + 0) = (u_encoder0_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder0_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder0_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder1_pos;
      u_encoder1_pos.real = this->encoder1_pos;
      *(outbuffer + offset + 0) = (u_encoder1_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder1_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder1_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder2_pos;
      u_encoder2_pos.real = this->encoder2_pos;
      *(outbuffer + offset + 0) = (u_encoder2_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder2_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder2_pos);
      union {
        int32_t real;
        uint32_t base;
      } u_reg0_pos;
      u_reg0_pos.real = this->reg0_pos;
      *(outbuffer + offset + 0) = (u_reg0_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reg0_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reg0_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reg0_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reg0_pos);
      union {
        int32_t real;
        uint32_t base;
      } u_reg1_pos;
      u_reg1_pos.real = this->reg1_pos;
      *(outbuffer + offset + 0) = (u_reg1_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reg1_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reg1_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reg1_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reg1_pos);
      union {
        int32_t real;
        uint32_t base;
      } u_reg2_pos;
      u_reg2_pos.real = this->reg2_pos;
      *(outbuffer + offset + 0) = (u_reg2_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reg2_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reg2_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reg2_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reg2_pos);
      *(outbuffer + offset + 0) = (this->driverPositionError >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->driverPositionError >> (8 * 1)) & 0xFF;
      offset += sizeof(this->driverPositionError);
      union {
        bool real;
        uint8_t base;
      } u_isStopped;
      u_isStopped.real = this->isStopped;
      *(outbuffer + offset + 0) = (u_isStopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isStopped);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_encoder0_pos;
      u_encoder0_pos.base = 0;
      u_encoder0_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder0_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder0_pos = u_encoder0_pos.real;
      offset += sizeof(this->encoder0_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder1_pos;
      u_encoder1_pos.base = 0;
      u_encoder1_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder1_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder1_pos = u_encoder1_pos.real;
      offset += sizeof(this->encoder1_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder2_pos;
      u_encoder2_pos.base = 0;
      u_encoder2_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder2_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder2_pos = u_encoder2_pos.real;
      offset += sizeof(this->encoder2_pos);
      union {
        int32_t real;
        uint32_t base;
      } u_reg0_pos;
      u_reg0_pos.base = 0;
      u_reg0_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reg0_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reg0_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reg0_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reg0_pos = u_reg0_pos.real;
      offset += sizeof(this->reg0_pos);
      union {
        int32_t real;
        uint32_t base;
      } u_reg1_pos;
      u_reg1_pos.base = 0;
      u_reg1_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reg1_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reg1_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reg1_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reg1_pos = u_reg1_pos.real;
      offset += sizeof(this->reg1_pos);
      union {
        int32_t real;
        uint32_t base;
      } u_reg2_pos;
      u_reg2_pos.base = 0;
      u_reg2_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reg2_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reg2_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reg2_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reg2_pos = u_reg2_pos.real;
      offset += sizeof(this->reg2_pos);
      this->driverPositionError =  ((uint16_t) (*(inbuffer + offset)));
      this->driverPositionError |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->driverPositionError);
      union {
        bool real;
        uint8_t base;
      } u_isStopped;
      u_isStopped.base = 0;
      u_isStopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isStopped = u_isStopped.real;
      offset += sizeof(this->isStopped);
     return offset;
    }

    const char * getType(){ return "bombel_msgs/BombelState"; };
    const char * getMD5(){ return "07fd29de5997ed81c83e7bd1d7e1df33"; };

  };

}
#endif
