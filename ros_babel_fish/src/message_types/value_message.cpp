// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/message_types/value_message.h"

namespace ros_babel_fish
{

//! ============== Time Specialization ==============

template<>
ros::Time ValueMessage<ros::Time>::getValue() const
{
  if ( from_stream_ )
  {
    uint32_t secs = *reinterpret_cast<const uint32_t *>(stream_);
    uint32_t nsecs = *reinterpret_cast<const uint32_t *>(stream_ + 4);
    return { secs, nsecs };
  }
  return value_;
}

template<>
size_t ValueMessage<ros::Time>::size() const { return 8; }


template<>
ValueMessage<ros::Time> *ValueMessage<ros::Time>::fromData( const uint8_t *data, size_t &bytes_read )
{
  uint32_t secs = *reinterpret_cast<const uint32_t *>(data);
  uint32_t nsecs = *reinterpret_cast<const uint32_t *>(data + 4);
  bytes_read = 8;
  return new ValueMessage<ros::Time>( ros::Time( secs, nsecs ));
}

template<>
size_t ValueMessage<ros::Time>::writeToStream( uint8_t *stream ) const
{
  if ( from_stream_ )
  {
    std::memcpy( stream, stream_, 8 );
    return 8;
  }
  *reinterpret_cast<uint32_t *>(stream) = value_.sec;
  *reinterpret_cast<uint32_t *>(stream + 4) = value_.nsec;
  return 8;
}

//! ============== Duration Specialization ==============

template<>
ros::Duration ValueMessage<ros::Duration>::getValue() const
{
  if ( from_stream_ )
  {
    int32_t secs = *reinterpret_cast<const int32_t *>(stream_);
    int32_t nsecs = *reinterpret_cast<const int32_t *>(stream_ + 4);
    return { secs, nsecs };
  }
  return value_;
}

template<>
size_t ValueMessage<ros::Duration>::size() const { return 8; }


template<>
ValueMessage<ros::Duration> *ValueMessage<ros::Duration>::fromData( const uint8_t *data, size_t &bytes_read )
{
  int32_t secs = *reinterpret_cast<const int32_t *>(data);
  int32_t nsecs = *reinterpret_cast<const int32_t *>(data + 4);
  bytes_read = 8;
  return new ValueMessage<ros::Duration>( ros::Duration( secs, nsecs ));
}

template<>
size_t ValueMessage<ros::Duration>::writeToStream( uint8_t *stream ) const
{
  if ( from_stream_ )
  {
    std::memcpy( stream, stream_, 8 );
    return 8;
  }
  *reinterpret_cast<int32_t *>(stream) = value_.sec;
  *reinterpret_cast<int32_t *>(stream + 4) = value_.nsec;
  return 8;
}

//! ============== String Specialization ==============

template<>
std::string ValueMessage<std::string>::getValue() const
{
  // Lazy copy only if needed but in the stream the string is not 0 terminated
  if ( from_stream_ )
  {
    uint32_t len = *reinterpret_cast<const uint32_t *>(stream_);
    return std::string( reinterpret_cast<const char *>(stream_ + 4), len );
  }
  return value_;
}

template<>
size_t ValueMessage<std::string>::size() const
{
  if ( from_stream_ ) return *reinterpret_cast<const uint32_t *>(stream_) + 4;
  return value_.length() + 4;
}

template<>
ValueMessage<std::string> *ValueMessage<std::string>::fromData( const uint8_t *data, size_t &bytes_read )
{
  uint32_t len = *reinterpret_cast<const uint32_t *>(data);
  bytes_read = len + 4;
  return new ValueMessage<std::string>( data );
}

template<>
size_t ValueMessage<std::string>::writeToStream( uint8_t *stream ) const
{
  if ( from_stream_ )
  {
    uint32_t len = *reinterpret_cast<const uint32_t *>(stream_) + 4;
    std::memcpy( stream, stream_, len );
    return len;
  }
  *reinterpret_cast<uint32_t *>(stream) = value_.length();
  stream += 4;
  memcpy( stream, value_.data(), value_.length());
  return value_.length() + 4;
}
}
