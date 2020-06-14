#ifndef _ROS_kobuki_msgs_AutoDockingActionGoal_h
#define _ROS_kobuki_msgs_AutoDockingActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "kobuki_msgs/AutoDockingGoal.h"

namespace kobuki_msgs
{

  class AutoDockingActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef kobuki_msgs::AutoDockingGoal _goal_type;
      _goal_type goal;

    AutoDockingActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "kobuki_msgs/AutoDockingActionGoal"; };
    const char * getMD5(){ return "4b30be6cd12b9e72826df56b481f40e0"; };

  };

}
#endif
