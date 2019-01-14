#ifndef _ROS_bombel_msgs_BombelSpeed_h
#define _ROS_bombel_msgs_BombelSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bombel_msgs
{

  class BombelSpeed : public ros::Msg
  {
    public:
      typedef int16_t _seq_type;
      _seq_type seq;
      typedef int16_t _joint0_speed_type;
      _joint0_speed_type joint0_speed;
      typedef int16_t _joint1_speed_type;
      _joint1_speed_type joint1_speed;
      typedef int16_t _joint2_speed_type;
      _joint2_speed_type joint2_speed;

    BombelSpeed():
      seq(0),
      joint0_speed(0),
      joint1_speed(0),
      joint2_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_seq;
      u_seq.real = this->seq;
      *(outbuffer + offset + 0) = (u_seq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_seq.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->seq);
      union {
        int16_t real;
        uint16_t base;
      } u_joint0_speed;
      u_joint0_speed.real = this->joint0_speed;
      *(outbuffer + offset + 0) = (u_joint0_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint0_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->joint0_speed);
      union {
        int16_t real;
        uint16_t base;
      } u_joint1_speed;
      u_joint1_speed.real = this->joint1_speed;
      *(outbuffer + offset + 0) = (u_joint1_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint1_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->joint1_speed);
      union {
        int16_t real;
        uint16_t base;
      } u_joint2_speed;
      u_joint2_speed.real = this->joint2_speed;
      *(outbuffer + offset + 0) = (u_joint2_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint2_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->joint2_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_seq;
      u_seq.base = 0;
      u_seq.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_seq.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq = u_seq.real;
      offset += sizeof(this->seq);
      union {
        int16_t real;
        uint16_t base;
      } u_joint0_speed;
      u_joint0_speed.base = 0;
      u_joint0_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint0_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint0_speed = u_joint0_speed.real;
      offset += sizeof(this->joint0_speed);
      union {
        int16_t real;
        uint16_t base;
      } u_joint1_speed;
      u_joint1_speed.base = 0;
      u_joint1_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint1_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint1_speed = u_joint1_speed.real;
      offset += sizeof(this->joint1_speed);
      union {
        int16_t real;
        uint16_t base;
      } u_joint2_speed;
      u_joint2_speed.base = 0;
      u_joint2_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint2_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint2_speed = u_joint2_speed.real;
      offset += sizeof(this->joint2_speed);
     return offset;
    }

    const char * getType(){ return "bombel_msgs/BombelSpeed"; };
    const char * getMD5(){ return "ecbd264165b80fd27334015e7121f238"; };

  };

}
#endif
