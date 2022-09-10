#ifndef _ROS_controller_msgs_FlatTarget_h
#define _ROS_controller_msgs_FlatTarget_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace controller_msgs
{

  class FlatTarget : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _type_mask_type;
      _type_mask_type type_mask;
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      typedef geometry_msgs::Vector3 _acceleration_type;
      _acceleration_type acceleration;
      typedef geometry_msgs::Vector3 _jerk_type;
      _jerk_type jerk;
      typedef geometry_msgs::Vector3 _snap_type;
      _snap_type snap;
      enum { IGNORE_SNAP =  1	 };
      enum { IGNORE_SNAP_JERK =  2	 };
      enum { IGNORE_SNAP_JERK_ACC =  4	 };

    FlatTarget():
      header(),
      type_mask(0),
      position(),
      velocity(),
      acceleration(),
      jerk(),
      snap()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->type_mask >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type_mask);
      offset += this->position.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->acceleration.serialize(outbuffer + offset);
      offset += this->jerk.serialize(outbuffer + offset);
      offset += this->snap.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->type_mask =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type_mask);
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->acceleration.deserialize(inbuffer + offset);
      offset += this->jerk.deserialize(inbuffer + offset);
      offset += this->snap.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "controller_msgs/FlatTarget"; };
    virtual const char * getMD5() override { return "c62e72c40204947fb0e6a39a53afefad"; };

  };

}
#endif
