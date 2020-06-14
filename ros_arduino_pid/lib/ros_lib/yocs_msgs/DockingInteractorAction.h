#ifndef _ROS_yocs_msgs_DockingInteractorAction_h
#define _ROS_yocs_msgs_DockingInteractorAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "yocs_msgs/DockingInteractorActionGoal.h"
#include "yocs_msgs/DockingInteractorActionResult.h"
#include "yocs_msgs/DockingInteractorActionFeedback.h"

namespace yocs_msgs
{

  class DockingInteractorAction : public ros::Msg
  {
    public:
      typedef yocs_msgs::DockingInteractorActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef yocs_msgs::DockingInteractorActionResult _action_result_type;
      _action_result_type action_result;
      typedef yocs_msgs::DockingInteractorActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    DockingInteractorAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "yocs_msgs/DockingInteractorAction"; };
    const char * getMD5(){ return "fb33b77bf3a235c893241fe6926c99eb"; };

  };

}
#endif
