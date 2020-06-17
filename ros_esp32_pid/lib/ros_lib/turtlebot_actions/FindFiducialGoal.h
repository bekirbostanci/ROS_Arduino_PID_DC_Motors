#ifndef _ROS_turtlebot_actions_FindFiducialGoal_h
#define _ROS_turtlebot_actions_FindFiducialGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot_actions
{

  class FindFiducialGoal : public ros::Msg
  {
    public:
      typedef const char* _camera_name_type;
      _camera_name_type camera_name;
      typedef uint8_t _pattern_width_type;
      _pattern_width_type pattern_width;
      typedef uint8_t _pattern_height_type;
      _pattern_height_type pattern_height;
      typedef float _pattern_size_type;
      _pattern_size_type pattern_size;
      typedef uint8_t _pattern_type_type;
      _pattern_type_type pattern_type;
      enum { CHESSBOARD =  1 };
      enum { CIRCLES_GRID =  2 };
      enum { ASYMMETRIC_CIRCLES_GRID = 3 };

    FindFiducialGoal():
      camera_name(""),
      pattern_width(0),
      pattern_height(0),
      pattern_size(0),
      pattern_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_camera_name = strlen(this->camera_name);
      varToArr(outbuffer + offset, length_camera_name);
      offset += 4;
      memcpy(outbuffer + offset, this->camera_name, length_camera_name);
      offset += length_camera_name;
      *(outbuffer + offset + 0) = (this->pattern_width >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pattern_width);
      *(outbuffer + offset + 0) = (this->pattern_height >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pattern_height);
      union {
        float real;
        uint32_t base;
      } u_pattern_size;
      u_pattern_size.real = this->pattern_size;
      *(outbuffer + offset + 0) = (u_pattern_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pattern_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pattern_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pattern_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pattern_size);
      *(outbuffer + offset + 0) = (this->pattern_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pattern_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_camera_name;
      arrToVar(length_camera_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_camera_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_camera_name-1]=0;
      this->camera_name = (char *)(inbuffer + offset-1);
      offset += length_camera_name;
      this->pattern_width =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pattern_width);
      this->pattern_height =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pattern_height);
      union {
        float real;
        uint32_t base;
      } u_pattern_size;
      u_pattern_size.base = 0;
      u_pattern_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pattern_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pattern_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pattern_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pattern_size = u_pattern_size.real;
      offset += sizeof(this->pattern_size);
      this->pattern_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pattern_type);
     return offset;
    }

    const char * getType(){ return "turtlebot_actions/FindFiducialGoal"; };
    const char * getMD5(){ return "8906385fe785bb5733551eb61968fe5b"; };

  };

}
#endif
