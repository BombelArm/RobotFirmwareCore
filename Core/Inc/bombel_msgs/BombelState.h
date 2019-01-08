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

    BombelState():
      encoder0_pos(0),
      encoder1_pos(0),
      encoder2_pos(0)
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
     return offset;
    }

    const char * getType(){ return "bombel_msgs/BombelState"; };
    const char * getMD5(){ return "df7941c35dba23e61d9274c9e9c86816"; };

  };

}
#endif
