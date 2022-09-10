#ifndef _ROS_delta_2_ServoAngles6DoFStamped_h
#define _ROS_delta_2_ServoAngles6DoFStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace delta_2
{

  class ServoAngles6DoFStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _Theta1_type;
      _Theta1_type Theta1;
      typedef double _Theta2_type;
      _Theta2_type Theta2;
      typedef double _Theta3_type;
      _Theta3_type Theta3;
      typedef double _Theta4_type;
      _Theta4_type Theta4;
      typedef double _Theta5_type;
      _Theta5_type Theta5;
      typedef double _Theta6_type;
      _Theta6_type Theta6;

    ServoAngles6DoFStamped():
      header(),
      Theta1(0),
      Theta2(0),
      Theta3(0),
      Theta4(0),
      Theta5(0),
      Theta6(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_Theta1;
      u_Theta1.real = this->Theta1;
      *(outbuffer + offset + 0) = (u_Theta1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Theta1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Theta1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Theta1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Theta1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Theta1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Theta1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Theta1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Theta1);
      union {
        double real;
        uint64_t base;
      } u_Theta2;
      u_Theta2.real = this->Theta2;
      *(outbuffer + offset + 0) = (u_Theta2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Theta2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Theta2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Theta2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Theta2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Theta2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Theta2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Theta2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Theta2);
      union {
        double real;
        uint64_t base;
      } u_Theta3;
      u_Theta3.real = this->Theta3;
      *(outbuffer + offset + 0) = (u_Theta3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Theta3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Theta3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Theta3.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Theta3.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Theta3.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Theta3.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Theta3.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Theta3);
      union {
        double real;
        uint64_t base;
      } u_Theta4;
      u_Theta4.real = this->Theta4;
      *(outbuffer + offset + 0) = (u_Theta4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Theta4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Theta4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Theta4.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Theta4.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Theta4.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Theta4.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Theta4.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Theta4);
      union {
        double real;
        uint64_t base;
      } u_Theta5;
      u_Theta5.real = this->Theta5;
      *(outbuffer + offset + 0) = (u_Theta5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Theta5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Theta5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Theta5.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Theta5.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Theta5.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Theta5.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Theta5.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Theta5);
      union {
        double real;
        uint64_t base;
      } u_Theta6;
      u_Theta6.real = this->Theta6;
      *(outbuffer + offset + 0) = (u_Theta6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Theta6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Theta6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Theta6.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Theta6.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Theta6.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Theta6.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Theta6.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Theta6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_Theta1;
      u_Theta1.base = 0;
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Theta1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Theta1 = u_Theta1.real;
      offset += sizeof(this->Theta1);
      union {
        double real;
        uint64_t base;
      } u_Theta2;
      u_Theta2.base = 0;
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Theta2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Theta2 = u_Theta2.real;
      offset += sizeof(this->Theta2);
      union {
        double real;
        uint64_t base;
      } u_Theta3;
      u_Theta3.base = 0;
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Theta3.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Theta3 = u_Theta3.real;
      offset += sizeof(this->Theta3);
      union {
        double real;
        uint64_t base;
      } u_Theta4;
      u_Theta4.base = 0;
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Theta4.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Theta4 = u_Theta4.real;
      offset += sizeof(this->Theta4);
      union {
        double real;
        uint64_t base;
      } u_Theta5;
      u_Theta5.base = 0;
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Theta5.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Theta5 = u_Theta5.real;
      offset += sizeof(this->Theta5);
      union {
        double real;
        uint64_t base;
      } u_Theta6;
      u_Theta6.base = 0;
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Theta6.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Theta6 = u_Theta6.real;
      offset += sizeof(this->Theta6);
     return offset;
    }

    virtual const char * getType() override { return "delta_2/ServoAngles6DoFStamped"; };
    virtual const char * getMD5() override { return "5c0ccc679186f5e65bdbfb2265c5a5e0"; };

  };

}
#endif
