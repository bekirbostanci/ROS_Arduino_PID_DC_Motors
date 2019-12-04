#ifndef _ROS_yocs_msgs_DockingInteractorActionResult_h
#define _ROS_yocs_msgs_DockingInteractorActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "yocs_msgs/DockingInteractorResult.h"

namespace yocs_msgs
{

  class DockingInteractorActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef yocs_msgs::DockingInteractorResult _result_type;
      _result_type result;

    DockingInteractorActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "yocs_msgs/DockingInteractorActionResult"; };
    const char * getMD5(){ return "1471476fb0fc7d1ce25819ffcafc3f6d"; };

  };

}
#endif
