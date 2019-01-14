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
      typedef int16_t _last_order_received_type;
      _last_order_received_type last_order_received;
      typedef uint16_t _order_errors_type;
      _order_errors_type order_errors;
      typedef uint16_t _driver_errors_type;
      _driver_errors_type driver_errors;

    BombelState():
      encoder0_pos(0),
      encoder1_pos(0),
      encoder2_pos(0),
      last_order_received(0),
      order_errors(0),
      driver_errors(0)
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
        int16_t real;
        uint16_t base;
      } u_last_order_received;
      u_last_order_received.real = this->last_order_received;
      *(outbuffer + offset + 0) = (u_last_order_received.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_last_order_received.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->last_order_received);
      *(outbuffer + offset + 0) = (this->order_errors >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->order_errors >> (8 * 1)) & 0xFF;
      offset += sizeof(this->order_errors);
      *(outbuffer + offset + 0) = (this->driver_errors >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->driver_errors >> (8 * 1)) & 0xFF;
      offset += sizeof(this->driver_errors);
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
        int16_t real;
        uint16_t base;
      } u_last_order_received;
      u_last_order_received.base = 0;
      u_last_order_received.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_last_order_received.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_order_received = u_last_order_received.real;
      offset += sizeof(this->last_order_received);
      this->order_errors =  ((uint16_t) (*(inbuffer + offset)));
      this->order_errors |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->order_errors);
      this->driver_errors =  ((uint16_t) (*(inbuffer + offset)));
      this->driver_errors |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->driver_errors);
     return offset;
    }

    const char * getType(){ return "bombel_msgs/BombelState"; };
    const char * getMD5(){ return "fe10011d68333d88599d0622c2d6631c"; };

  };

}
#endif
