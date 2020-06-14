#ifndef _ROS_yocs_msgs_TrajectoryList_h
#define _ROS_yocs_msgs_TrajectoryList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "yocs_msgs/Trajectory.h"

namespace yocs_msgs
{

  class TrajectoryList : public ros::Msg
  {
    public:
      uint32_t trajectories_length;
      typedef yocs_msgs::Trajectory _trajectories_type;
      _trajectories_type st_trajectories;
      _trajectories_type * trajectories;

    TrajectoryList():
      trajectories_length(0), trajectories(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->trajectories_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectories_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectories_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectories_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectories_length);
      for( uint32_t i = 0; i < trajectories_length; i++){
      offset += this->trajectories[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t trajectories_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->trajectories_length);
      if(trajectories_lengthT > trajectories_length)
        this->trajectories = (yocs_msgs::Trajectory*)realloc(this->trajectories, trajectories_lengthT * sizeof(yocs_msgs::Trajectory));
      trajectories_length = trajectories_lengthT;
      for( uint32_t i = 0; i < trajectories_length; i++){
      offset += this->st_trajectories.deserialize(inbuffer + offset);
        memcpy( &(this->trajectories[i]), &(this->st_trajectories), sizeof(yocs_msgs::Trajectory));
      }
     return offset;
    }

    const char * getType(){ return "yocs_msgs/TrajectoryList"; };
    const char * getMD5(){ return "f0901d378c8ac2d2d3d8feafaa343a58"; };

  };

}
#endif
