#ifndef _ROS_yocs_msgs_ARPairList_h
#define _ROS_yocs_msgs_ARPairList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "yocs_msgs/ARPair.h"

namespace yocs_msgs
{

  class ARPairList : public ros::Msg
  {
    public:
      uint32_t pairs_length;
      typedef yocs_msgs::ARPair _pairs_type;
      _pairs_type st_pairs;
      _pairs_type * pairs;

    ARPairList():
      pairs_length(0), pairs(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pairs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pairs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pairs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pairs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pairs_length);
      for( uint32_t i = 0; i < pairs_length; i++){
      offset += this->pairs[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t pairs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pairs_length);
      if(pairs_lengthT > pairs_length)
        this->pairs = (yocs_msgs::ARPair*)realloc(this->pairs, pairs_lengthT * sizeof(yocs_msgs::ARPair));
      pairs_length = pairs_lengthT;
      for( uint32_t i = 0; i < pairs_length; i++){
      offset += this->st_pairs.deserialize(inbuffer + offset);
        memcpy( &(this->pairs[i]), &(this->st_pairs), sizeof(yocs_msgs::ARPair));
      }
     return offset;
    }

    const char * getType(){ return "yocs_msgs/ARPairList"; };
    const char * getMD5(){ return "0d0cfdee6418b6b3edd41c0abc93d76f"; };

  };

}
#endif
