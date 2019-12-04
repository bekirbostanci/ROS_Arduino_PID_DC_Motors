#ifndef _ROS_stdr_msgs_RegisterRobotResult_h
#define _ROS_stdr_msgs_RegisterRobotResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "stdr_msgs/RobotMsg.h"

namespace stdr_msgs
{

  class RegisterRobotResult : public ros::Msg
  {
    public:
      typedef stdr_msgs::RobotMsg _description_type;
      _description_type description;

    RegisterRobotResult():
      description()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->description.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->description.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "stdr_msgs/RegisterRobotResult"; };
    const char * getMD5(){ return "918af704950f3898fd078737c940764a"; };

  };

}
#endif
