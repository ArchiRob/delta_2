#ifndef _ROS_SERVICE_CalcTrajectory_h
#define _ROS_SERVICE_CalcTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"

namespace tricopter
{

static const char CALCTRAJECTORY[] = "tricopter/CalcTrajectory";

  class CalcTrajectoryRequest : public ros::Msg
  {
    public:

    CalcTrajectoryRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return CALCTRAJECTORY; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CalcTrajectoryResponse : public ros::Msg
  {
    public:
      typedef trajectory_msgs::MultiDOFJointTrajectory _trajectory_type;
      _trajectory_type trajectory;

    CalcTrajectoryResponse():
      trajectory()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->trajectory.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->trajectory.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return CALCTRAJECTORY; };
    virtual const char * getMD5() override { return "f517c0919226c86aaea44f94f6c54bf9"; };

  };

  class CalcTrajectory {
    public:
    typedef CalcTrajectoryRequest Request;
    typedef CalcTrajectoryResponse Response;
  };

}
#endif
