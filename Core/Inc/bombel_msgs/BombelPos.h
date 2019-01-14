#ifndef _ROS_bombel_msgs_BombelPos_h
#define _ROS_bombel_msgs_BombelPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bombel_msgs
{

  class BombelPos : public ros::Msg
  {
    public:
      typedef int16_t _seq_type;
      _seq_type seq;
      typedef float _joint0_pos_type;
      _joint0_pos_type joint0_pos;
      typedef float _joint1_pos_type;
      _joint1_pos_type joint1_pos;
      typedef float _joint2_pos_type;
      _joint2_pos_type joint2_pos;

    BombelPos():
      seq(0),
      joint0_pos(0),
      joint1_pos(0),
      joint2_pos(0)
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
        float real;
        uint32_t base;
      } u_joint0_pos;
      u_joint0_pos.real = this->joint0_pos;
      *(outbuffer + offset + 0) = (u_joint0_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint0_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint0_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint0_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint0_pos);
      union {
        float real;
        uint32_t base;
      } u_joint1_pos;
      u_joint1_pos.real = this->joint1_pos;
      *(outbuffer + offset + 0) = (u_joint1_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint1_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint1_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint1_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint1_pos);
      union {
        float real;
        uint32_t base;
      } u_joint2_pos;
      u_joint2_pos.real = this->joint2_pos;
      *(outbuffer + offset + 0) = (u_joint2_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint2_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint2_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint2_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint2_pos);
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
        float real;
        uint32_t base;
      } u_joint0_pos;
      u_joint0_pos.base = 0;
      u_joint0_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint0_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint0_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint0_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint0_pos = u_joint0_pos.real;
      offset += sizeof(this->joint0_pos);
      union {
        float real;
        uint32_t base;
      } u_joint1_pos;
      u_joint1_pos.base = 0;
      u_joint1_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint1_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint1_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint1_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint1_pos = u_joint1_pos.real;
      offset += sizeof(this->joint1_pos);
      union {
        float real;
        uint32_t base;
      } u_joint2_pos;
      u_joint2_pos.base = 0;
      u_joint2_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint2_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint2_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint2_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint2_pos = u_joint2_pos.real;
      offset += sizeof(this->joint2_pos);
     return offset;
    }

    const char * getType(){ return "bombel_msgs/BombelPos"; };
    const char * getMD5(){ return "9ef486ae0fd99e4ec0734e613cb7b489"; };

  };

}
#endif
