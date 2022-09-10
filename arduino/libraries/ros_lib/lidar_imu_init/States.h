#ifndef _ROS_lidar_imu_init_States_h
#define _ROS_lidar_imu_init_States_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace lidar_imu_init
{

  class States : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t rot_end_length;
      typedef double _rot_end_type;
      _rot_end_type st_rot_end;
      _rot_end_type * rot_end;
      uint32_t pos_end_length;
      typedef double _pos_end_type;
      _pos_end_type st_pos_end;
      _pos_end_type * pos_end;
      uint32_t vel_end_length;
      typedef double _vel_end_type;
      _vel_end_type st_vel_end;
      _vel_end_type * vel_end;
      uint32_t bias_gyr_length;
      typedef double _bias_gyr_type;
      _bias_gyr_type st_bias_gyr;
      _bias_gyr_type * bias_gyr;
      uint32_t bias_acc_length;
      typedef double _bias_acc_type;
      _bias_acc_type st_bias_acc;
      _bias_acc_type * bias_acc;
      uint32_t gravity_length;
      typedef double _gravity_type;
      _gravity_type st_gravity;
      _gravity_type * gravity;
      uint32_t cov_length;
      typedef double _cov_type;
      _cov_type st_cov;
      _cov_type * cov;

    States():
      header(),
      rot_end_length(0), st_rot_end(), rot_end(nullptr),
      pos_end_length(0), st_pos_end(), pos_end(nullptr),
      vel_end_length(0), st_vel_end(), vel_end(nullptr),
      bias_gyr_length(0), st_bias_gyr(), bias_gyr(nullptr),
      bias_acc_length(0), st_bias_acc(), bias_acc(nullptr),
      gravity_length(0), st_gravity(), gravity(nullptr),
      cov_length(0), st_cov(), cov(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->rot_end_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rot_end_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rot_end_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rot_end_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rot_end_length);
      for( uint32_t i = 0; i < rot_end_length; i++){
      union {
        double real;
        uint64_t base;
      } u_rot_endi;
      u_rot_endi.real = this->rot_end[i];
      *(outbuffer + offset + 0) = (u_rot_endi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rot_endi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rot_endi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rot_endi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rot_endi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rot_endi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rot_endi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rot_endi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rot_end[i]);
      }
      *(outbuffer + offset + 0) = (this->pos_end_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pos_end_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pos_end_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pos_end_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_end_length);
      for( uint32_t i = 0; i < pos_end_length; i++){
      union {
        double real;
        uint64_t base;
      } u_pos_endi;
      u_pos_endi.real = this->pos_end[i];
      *(outbuffer + offset + 0) = (u_pos_endi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_endi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_endi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_endi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pos_endi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pos_endi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pos_endi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pos_endi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pos_end[i]);
      }
      *(outbuffer + offset + 0) = (this->vel_end_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vel_end_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vel_end_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vel_end_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_end_length);
      for( uint32_t i = 0; i < vel_end_length; i++){
      union {
        double real;
        uint64_t base;
      } u_vel_endi;
      u_vel_endi.real = this->vel_end[i];
      *(outbuffer + offset + 0) = (u_vel_endi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_endi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_endi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_endi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vel_endi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vel_endi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vel_endi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vel_endi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vel_end[i]);
      }
      *(outbuffer + offset + 0) = (this->bias_gyr_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bias_gyr_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bias_gyr_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bias_gyr_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bias_gyr_length);
      for( uint32_t i = 0; i < bias_gyr_length; i++){
      union {
        double real;
        uint64_t base;
      } u_bias_gyri;
      u_bias_gyri.real = this->bias_gyr[i];
      *(outbuffer + offset + 0) = (u_bias_gyri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bias_gyri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bias_gyri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bias_gyri.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bias_gyri.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bias_gyri.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bias_gyri.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bias_gyri.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bias_gyr[i]);
      }
      *(outbuffer + offset + 0) = (this->bias_acc_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bias_acc_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bias_acc_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bias_acc_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bias_acc_length);
      for( uint32_t i = 0; i < bias_acc_length; i++){
      union {
        double real;
        uint64_t base;
      } u_bias_acci;
      u_bias_acci.real = this->bias_acc[i];
      *(outbuffer + offset + 0) = (u_bias_acci.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bias_acci.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bias_acci.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bias_acci.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bias_acci.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bias_acci.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bias_acci.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bias_acci.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bias_acc[i]);
      }
      *(outbuffer + offset + 0) = (this->gravity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gravity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gravity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gravity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gravity_length);
      for( uint32_t i = 0; i < gravity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_gravityi;
      u_gravityi.real = this->gravity[i];
      *(outbuffer + offset + 0) = (u_gravityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gravityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gravityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gravityi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gravityi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gravityi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gravityi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gravityi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gravity[i]);
      }
      *(outbuffer + offset + 0) = (this->cov_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cov_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cov_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cov_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cov_length);
      for( uint32_t i = 0; i < cov_length; i++){
      union {
        double real;
        uint64_t base;
      } u_covi;
      u_covi.real = this->cov[i];
      *(outbuffer + offset + 0) = (u_covi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_covi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_covi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_covi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_covi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_covi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_covi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_covi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->cov[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t rot_end_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rot_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rot_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rot_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rot_end_length);
      if(rot_end_lengthT > rot_end_length)
        this->rot_end = (double*)realloc(this->rot_end, rot_end_lengthT * sizeof(double));
      rot_end_length = rot_end_lengthT;
      for( uint32_t i = 0; i < rot_end_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_rot_end;
      u_st_rot_end.base = 0;
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_rot_end.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_rot_end = u_st_rot_end.real;
      offset += sizeof(this->st_rot_end);
        memcpy( &(this->rot_end[i]), &(this->st_rot_end), sizeof(double));
      }
      uint32_t pos_end_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pos_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pos_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pos_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pos_end_length);
      if(pos_end_lengthT > pos_end_length)
        this->pos_end = (double*)realloc(this->pos_end, pos_end_lengthT * sizeof(double));
      pos_end_length = pos_end_lengthT;
      for( uint32_t i = 0; i < pos_end_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_pos_end;
      u_st_pos_end.base = 0;
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_pos_end.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_pos_end = u_st_pos_end.real;
      offset += sizeof(this->st_pos_end);
        memcpy( &(this->pos_end[i]), &(this->st_pos_end), sizeof(double));
      }
      uint32_t vel_end_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vel_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vel_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vel_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vel_end_length);
      if(vel_end_lengthT > vel_end_length)
        this->vel_end = (double*)realloc(this->vel_end, vel_end_lengthT * sizeof(double));
      vel_end_length = vel_end_lengthT;
      for( uint32_t i = 0; i < vel_end_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_vel_end;
      u_st_vel_end.base = 0;
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_vel_end.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_vel_end = u_st_vel_end.real;
      offset += sizeof(this->st_vel_end);
        memcpy( &(this->vel_end[i]), &(this->st_vel_end), sizeof(double));
      }
      uint32_t bias_gyr_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bias_gyr_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bias_gyr_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bias_gyr_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bias_gyr_length);
      if(bias_gyr_lengthT > bias_gyr_length)
        this->bias_gyr = (double*)realloc(this->bias_gyr, bias_gyr_lengthT * sizeof(double));
      bias_gyr_length = bias_gyr_lengthT;
      for( uint32_t i = 0; i < bias_gyr_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_bias_gyr;
      u_st_bias_gyr.base = 0;
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_bias_gyr.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_bias_gyr = u_st_bias_gyr.real;
      offset += sizeof(this->st_bias_gyr);
        memcpy( &(this->bias_gyr[i]), &(this->st_bias_gyr), sizeof(double));
      }
      uint32_t bias_acc_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bias_acc_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bias_acc_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bias_acc_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bias_acc_length);
      if(bias_acc_lengthT > bias_acc_length)
        this->bias_acc = (double*)realloc(this->bias_acc, bias_acc_lengthT * sizeof(double));
      bias_acc_length = bias_acc_lengthT;
      for( uint32_t i = 0; i < bias_acc_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_bias_acc;
      u_st_bias_acc.base = 0;
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_bias_acc.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_bias_acc = u_st_bias_acc.real;
      offset += sizeof(this->st_bias_acc);
        memcpy( &(this->bias_acc[i]), &(this->st_bias_acc), sizeof(double));
      }
      uint32_t gravity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      gravity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      gravity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      gravity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->gravity_length);
      if(gravity_lengthT > gravity_length)
        this->gravity = (double*)realloc(this->gravity, gravity_lengthT * sizeof(double));
      gravity_length = gravity_lengthT;
      for( uint32_t i = 0; i < gravity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_gravity;
      u_st_gravity.base = 0;
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_gravity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_gravity = u_st_gravity.real;
      offset += sizeof(this->st_gravity);
        memcpy( &(this->gravity[i]), &(this->st_gravity), sizeof(double));
      }
      uint32_t cov_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cov_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cov_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cov_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cov_length);
      if(cov_lengthT > cov_length)
        this->cov = (double*)realloc(this->cov, cov_lengthT * sizeof(double));
      cov_length = cov_lengthT;
      for( uint32_t i = 0; i < cov_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_cov;
      u_st_cov.base = 0;
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_cov.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_cov = u_st_cov.real;
      offset += sizeof(this->st_cov);
        memcpy( &(this->cov[i]), &(this->st_cov), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return "lidar_imu_init/States"; };
    virtual const char * getMD5() override { return "4a896a0d8c07506c836e98c3fa512a5e"; };

  };

}
#endif
