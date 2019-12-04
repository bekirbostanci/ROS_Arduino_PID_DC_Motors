#ifndef _ROS_yocs_msgs_ColumnList_h
#define _ROS_yocs_msgs_ColumnList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "yocs_msgs/Column.h"

namespace yocs_msgs
{

  class ColumnList : public ros::Msg
  {
    public:
      uint32_t obstacles_length;
      typedef yocs_msgs::Column _obstacles_type;
      _obstacles_type st_obstacles;
      _obstacles_type * obstacles;

    ColumnList():
      obstacles_length(0), obstacles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacles_length);
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t obstacles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obstacles_length);
      if(obstacles_lengthT > obstacles_length)
        this->obstacles = (yocs_msgs::Column*)realloc(this->obstacles, obstacles_lengthT * sizeof(yocs_msgs::Column));
      obstacles_length = obstacles_lengthT;
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->st_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->obstacles[i]), &(this->st_obstacles), sizeof(yocs_msgs::Column));
      }
     return offset;
    }

    const char * getType(){ return "yocs_msgs/ColumnList"; };
    const char * getMD5(){ return "fdf98d2a8a0cfe17000cdb66889826b6"; };

  };

}
#endif
