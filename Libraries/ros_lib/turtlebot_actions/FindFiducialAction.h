#ifndef _ROS_turtlebot_actions_FindFiducialAction_h
#define _ROS_turtlebot_actions_FindFiducialAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "turtlebot_actions/FindFiducialActionGoal.h"
#include "turtlebot_actions/FindFiducialActionResult.h"
#include "turtlebot_actions/FindFiducialActionFeedback.h"

namespace turtlebot_actions
{

  class FindFiducialAction : public ros::Msg
  {
    public:
      typedef turtlebot_actions::FindFiducialActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef turtlebot_actions::FindFiducialActionResult _action_result_type;
      _action_result_type action_result;
      typedef turtlebot_actions::FindFiducialActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    FindFiducialAction():
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

    const char * getType(){ return "turtlebot_actions/FindFiducialAction"; };
    const char * getMD5(){ return "87cc8981a7a81f16abc7af19809908dd"; };

  };

}
#endif
