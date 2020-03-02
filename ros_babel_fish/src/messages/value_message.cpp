// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/messages/value_message.h"

namespace ros_babel_fish
{

//! ============== Bool Specialization ==============

template<>
bool ValueMessage<bool>::getValue() const
{
  if ( from_stream_ )
  {
    return *reinterpret_cast<const uint8_t *>(stream_) != 0;
  }
  return value_;
}

template<>
size_t ValueMessage<bool>::_sizeInBytes() const { return 1; }

template<>
ValueMessage<bool> *ValueMessage<bool>::fromStream( const uint8_t *stream, size_t stream_length, size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  uint8_t val = *reinterpret_cast<const uint8_t *>(stream + bytes_read);
  ++bytes_read;
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
  return new ValueMessage<bool>( val != 0 );
}

template<>
size_t ValueMessage<bool>::writeToStream( uint8_t *stream ) const
{
  if ( from_stream_ )
  {
    *stream = *stream_;
    return 1;
  }
  *stream = static_cast<uint8_t >(value_ ? 1 : 0);
  return 1;
}

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
size_t ValueMessage<ros::Time>::_sizeInBytes() const { return 8; }


template<>
ValueMessage<ros::Time> *ValueMessage<ros::Time>::fromStream( const uint8_t *stream, size_t stream_length,
                                                              size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  uint32_t secs = *reinterpret_cast<const uint32_t *>(stream + bytes_read);
  uint32_t nsecs = *reinterpret_cast<const uint32_t *>(stream + bytes_read + 4);
  bytes_read += 8;
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
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
    int32_t nsecs = *reinterpret_cast<const int32_t *>(stream_ + sizeof( int32_t ));
    return { secs, nsecs };
  }
  return value_;
}

template<>
size_t ValueMessage<ros::Duration>::_sizeInBytes() const { return 8; }


template<>
ValueMessage<ros::Duration> *ValueMessage<ros::Duration>::fromStream( const uint8_t *stream, size_t stream_length,
                                                                      size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  int32_t secs = *reinterpret_cast<const int32_t *>(stream + bytes_read);
  int32_t nsecs = *reinterpret_cast<const int32_t *>(stream + bytes_read + sizeof( int32_t ));
  bytes_read += 8;
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
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
  *reinterpret_cast<int32_t *>(stream + sizeof( int32_t )) = value_.nsec;
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
size_t ValueMessage<std::string>::_sizeInBytes() const
{
  if ( from_stream_ ) return *reinterpret_cast<const uint32_t *>(stream_) + 4;
  return value_.length() + 4;
}

template<>
ValueMessage<std::string> *ValueMessage<std::string>::fromStream( const uint8_t *stream, size_t stream_length,
                                                                  size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  const uint8_t *begin = stream + bytes_read;
  uint32_t len = *reinterpret_cast<const uint32_t *>( begin );
  bytes_read += len + sizeof( uint32_t );
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
  return new ValueMessage<std::string>( begin );
}

template<>
size_t ValueMessage<std::string>::writeToStream( uint8_t *stream ) const
{
  if ( from_stream_ )
  {
    uint32_t len = *reinterpret_cast<const uint32_t *>(stream_) + sizeof( uint32_t );
    std::memcpy( stream, stream_, len );
    return len;
  }
  *reinterpret_cast<uint32_t *>(stream) = value_.length();
  stream += sizeof( uint32_t );
  memcpy( stream, value_.data(), value_.length());
  return value_.length() + sizeof( uint32_t );
}
}
