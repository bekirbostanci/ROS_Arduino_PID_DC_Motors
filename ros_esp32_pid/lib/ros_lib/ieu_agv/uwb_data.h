#ifndef _ROS_ieu_agv_uwb_data_h
#define _ROS_ieu_agv_uwb_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace ieu_agv
{

  class uwb_data : public ros::Msg
  {
    public:
      uint32_t destination_id_length;
      typedef int64_t _destination_id_type;
      _destination_id_type st_destination_id;
      _destination_id_type * destination_id;
      uint32_t distance_length;
      typedef float _distance_type;
      _distance_type st_distance;
      _distance_type * distance;
      uint32_t stamp_length;
      typedef ros::Time _stamp_type;
      _stamp_type st_stamp;
      _stamp_type * stamp;

    uwb_data():
      destination_id_length(0), destination_id(NULL),
      distance_length(0), distance(NULL),
      stamp_length(0), stamp(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->destination_id_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->destination_id_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->destination_id_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->destination_id_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->destination_id_length);
      for( uint32_t i = 0; i < destination_id_length; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_destination_idi;
      u_destination_idi.real = this->destination_id[i];
      *(outbuffer + offset + 0) = (u_destination_idi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_destination_idi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_destination_idi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_destination_idi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_destination_idi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_destination_idi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_destination_idi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_destination_idi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->destination_id[i]);
      }
      *(outbuffer + offset + 0) = (this->distance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->distance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->distance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_length);
      for( uint32_t i = 0; i < distance_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->distance[i]);
      }
      *(outbuffer + offset + 0) = (this->stamp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp_length);
      for( uint32_t i = 0; i < stamp_length; i++){
      *(outbuffer + offset + 0) = (this->stamp[i].sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp[i].sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp[i].sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp[i].sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp[i].sec);
      *(outbuffer + offset + 0) = (this->stamp[i].nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp[i].nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp[i].nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp[i].nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp[i].nsec);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t destination_id_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      destination_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      destination_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      destination_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->destination_id_length);
      if(destination_id_lengthT > destination_id_length)
        this->destination_id = (int64_t*)realloc(this->destination_id, destination_id_lengthT * sizeof(int64_t));
      destination_id_length = destination_id_lengthT;
      for( uint32_t i = 0; i < destination_id_length; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_st_destination_id;
      u_st_destination_id.base = 0;
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_destination_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_destination_id = u_st_destination_id.real;
      offset += sizeof(this->st_destination_id);
        memcpy( &(this->destination_id[i]), &(this->st_destination_id), sizeof(int64_t));
      }
      uint32_t distance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->distance_length);
      if(distance_lengthT > distance_length)
        this->distance = (float*)realloc(this->distance, distance_lengthT * sizeof(float));
      distance_length = distance_lengthT;
      for( uint32_t i = 0; i < distance_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_distance));
        memcpy( &(this->distance[i]), &(this->st_distance), sizeof(float));
      }
      uint32_t stamp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      stamp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      stamp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      stamp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->stamp_length);
      if(stamp_lengthT > stamp_length)
        this->stamp = (ros::Time*)realloc(this->stamp, stamp_lengthT * sizeof(ros::Time));
      stamp_length = stamp_lengthT;
      for( uint32_t i = 0; i < stamp_length; i++){
      this->st_stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_stamp.sec);
      this->st_stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_stamp.nsec);
        memcpy( &(this->stamp[i]), &(this->st_stamp), sizeof(ros::Time));
      }
     return offset;
    }

    const char * getType(){ return "ieu_agv/uwb_data"; };
    const char * getMD5(){ return "853a584c8bd9fd74a6b2709e39029b14"; };

  };

}
#endif
