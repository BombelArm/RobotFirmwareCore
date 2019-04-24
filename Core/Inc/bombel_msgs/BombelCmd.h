#ifndef _ROS_bombel_msgs_BombelCmd_h
#define _ROS_bombel_msgs_BombelCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bombel_msgs
{

  class BombelCmd : public ros::Msg
  {
    public:
      typedef uint16_t _seq_type;
      _seq_type seq;
      typedef uint16_t _cmd_type;
      _cmd_type cmd;
      typedef float _joint0_pos_type;
      _joint0_pos_type joint0_pos;
      typedef float _joint1_pos_type;
      _joint1_pos_type joint1_pos;
      typedef float _joint2_pos_type;
      _joint2_pos_type joint2_pos;

    BombelCmd():
      seq(0),
      cmd(0),
      joint0_pos(0),
      joint1_pos(0),
      joint2_pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq >> (8 * 1)) & 0xFF;
      offset += sizeof(this->seq);
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cmd >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cmd);
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
      this->seq =  ((uint16_t) (*(inbuffer + offset)));
      this->seq |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->seq);
      this->cmd =  ((uint16_t) (*(inbuffer + offset)));
      this->cmd |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->cmd);
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

    const char * getType(){ return "bombel_msgs/BombelCmd"; };
    const char * getMD5(){ return "1b1e87ad64a59979edd73f1306ebac50"; };

  };

}
#endif
