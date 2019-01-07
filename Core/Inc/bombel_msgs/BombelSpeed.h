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
      typedef uint32_t _joint0_speed_type;
      _joint0_speed_type joint0_speed;
      typedef uint32_t _joint1_speed_type;
      _joint1_speed_type joint1_speed;
      typedef uint32_t _joint2_speed_type;
      _joint2_speed_type joint2_speed;
      typedef uint32_t _joint3_speed_type;
      _joint3_speed_type joint3_speed;

    BombelSpeed():
      joint0_speed(0),
      joint1_speed(0),
      joint2_speed(0),
      joint3_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint0_speed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint0_speed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint0_speed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint0_speed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint0_speed);
      *(outbuffer + offset + 0) = (this->joint1_speed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint1_speed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint1_speed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint1_speed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint1_speed);
      *(outbuffer + offset + 0) = (this->joint2_speed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint2_speed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint2_speed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint2_speed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint2_speed);
      *(outbuffer + offset + 0) = (this->joint3_speed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint3_speed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint3_speed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint3_speed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint3_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->joint0_speed =  ((uint32_t) (*(inbuffer + offset)));
      this->joint0_speed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint0_speed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->joint0_speed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->joint0_speed);
      this->joint1_speed =  ((uint32_t) (*(inbuffer + offset)));
      this->joint1_speed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint1_speed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->joint1_speed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->joint1_speed);
      this->joint2_speed =  ((uint32_t) (*(inbuffer + offset)));
      this->joint2_speed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint2_speed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->joint2_speed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->joint2_speed);
      this->joint3_speed =  ((uint32_t) (*(inbuffer + offset)));
      this->joint3_speed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint3_speed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->joint3_speed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->joint3_speed);
     return offset;
    }

    const char * getType(){ return "bombel_msgs/BombelSpeed"; };
    const char * getMD5(){ return "80dcdbf5905ead8ad45545d1df7eb7ef"; };

  };

}
#endif
