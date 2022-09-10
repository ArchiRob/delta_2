#ifndef _ROS_contact_manipulator_servo_angles_h
#define _ROS_contact_manipulator_servo_angles_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace contact_manipulator
{

  class servo_angles : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _theta1_type;
      _theta1_type theta1;
      typedef int32_t _theta2_type;
      _theta2_type theta2;
      typedef int32_t _theta3_type;
      _theta3_type theta3;

    servo_angles():
      header(),
      theta1(0),
      theta2(0),
      theta3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_theta1;
      u_theta1.real = this->theta1;
      *(outbuffer + offset + 0) = (u_theta1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta1);
      union {
        int32_t real;
        uint32_t base;
      } u_theta2;
      u_theta2.real = this->theta2;
      *(outbuffer + offset + 0) = (u_theta2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta2);
      union {
        int32_t real;
        uint32_t base;
      } u_theta3;
      u_theta3.real = this->theta3;
      *(outbuffer + offset + 0) = (u_theta3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_theta1;
      u_theta1.base = 0;
      u_theta1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta1 = u_theta1.real;
      offset += sizeof(this->theta1);
      union {
        int32_t real;
        uint32_t base;
      } u_theta2;
      u_theta2.base = 0;
      u_theta2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta2 = u_theta2.real;
      offset += sizeof(this->theta2);
      union {
        int32_t real;
        uint32_t base;
      } u_theta3;
      u_theta3.base = 0;
      u_theta3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta3 = u_theta3.real;
      offset += sizeof(this->theta3);
     return offset;
    }

    virtual const char * getType() override { return "contact_manipulator/servo_angles"; };
    virtual const char * getMD5() override { return "b5872e8b88ee70f3255317405572beac"; };

  };

}
#endif
