#ifndef _ROS_yocs_msgs_TableList_h
#define _ROS_yocs_msgs_TableList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "yocs_msgs/Table.h"

namespace yocs_msgs
{

  class TableList : public ros::Msg
  {
    public:
      uint32_t tables_length;
      typedef yocs_msgs::Table _tables_type;
      _tables_type st_tables;
      _tables_type * tables;

    TableList():
      tables_length(0), tables(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->tables_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tables_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tables_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tables_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tables_length);
      for( uint32_t i = 0; i < tables_length; i++){
      offset += this->tables[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t tables_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tables_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tables_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tables_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tables_length);
      if(tables_lengthT > tables_length)
        this->tables = (yocs_msgs::Table*)realloc(this->tables, tables_lengthT * sizeof(yocs_msgs::Table));
      tables_length = tables_lengthT;
      for( uint32_t i = 0; i < tables_length; i++){
      offset += this->st_tables.deserialize(inbuffer + offset);
        memcpy( &(this->tables[i]), &(this->st_tables), sizeof(yocs_msgs::Table));
      }
     return offset;
    }

    const char * getType(){ return "yocs_msgs/TableList"; };
    const char * getMD5(){ return "88fff7e2df98bd37f03f85cffc55624b"; };

  };

}
#endif
