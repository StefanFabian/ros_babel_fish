// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/message_types/array_message.h"

namespace ros_babel_fish
{

//! ============== Message Specialization ==============

template<>
ArrayMessage<Message>::ArrayMessage( MessageType element_type, bool fixed_length, size_t length,
                                     const uint8_t *stream )
  : ArrayMessageBase( element_type, fixed_length, length, stream ), values_( length ), from_stream_( false ) { }

template<>
ArrayMessage<Message>::~ArrayMessage()
{
  for ( auto &entry : values_ )
  {
    delete entry;
  }
  values_.clear();
}

template<>
Message &ArrayMessage<Message>::operator[]( size_t index )
{
  return *values_[index];
}

template<>
const Message &ArrayMessage<Message>::operator[]( size_t index ) const
{
  return *values_[index];
}

template<>
size_t ArrayMessage<Message>::size() const
{
  size_t result = (fixed_length_ ? 0 : 4);
  for ( auto value : values_ )
  {
    result += value->size();
  }
  return result;
}

template<>
size_t ArrayMessage<Message>::writeToStream( uint8_t *stream ) const
{
  size_t offset = 0;
  if ( !fixed_length_ )
  {
    *reinterpret_cast<uint32_t *>(stream) = length_;
    offset = 4;
  }
  for ( auto &value : values_ )
  {
    offset += value->writeToStream( stream + offset );
  }
  return offset;
}


//! ============== Time Specialization ==============

template<>
ros::Time ArrayMessage<ros::Time>::operator[]( size_t index )
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    uint32_t secs = *reinterpret_cast<const uint32_t *>(stream_);
    uint32_t nsecs = *reinterpret_cast<const uint32_t *>(stream_ + 4);
    return { secs, nsecs };
  }
  return values_[index];
}

template<>
ros::Time ArrayMessage<ros::Time>::operator[]( size_t index ) const
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    uint32_t secs = *reinterpret_cast<const uint32_t *>(stream_);
    uint32_t nsecs = *reinterpret_cast<const uint32_t *>(stream_ + 4);
    return { secs, nsecs };
  }
  return values_[index];
}

template<>
size_t ArrayMessage<ros::Time>::size() const
{
  return 2 * sizeof( uint32_t ) * length_ + (fixed_length_ ? 0 : 4);
}

template<>
void ArrayMessage<ros::Time>::detachFromStream()
{
  if ( !from_stream_ ) return;
  auto data = reinterpret_cast<const uint32_t *>(stream_);
  values_.clear();
  for ( size_t i = 0; i < length_; ++i )
  {
    uint32_t secs = *data;
    ++data;
    uint32_t nsecs = *data;
    ++data;
    values_.emplace_back( secs, nsecs );
  }
  from_stream_ = false;
}

template<>
size_t ArrayMessage<ros::Time>::writeToStream( uint8_t *stream ) const
{
  size_t length = size();
  size_t count = length;
  if ( !fixed_length_ )
  {
    *reinterpret_cast<uint32_t *>(stream) = length_;
    stream += 4;
    count -= 4;
  }
  if ( from_stream_ )
  {
    std::memcpy( stream, stream_, count );
    return length;
  }
  auto *data = reinterpret_cast<uint32_t *>(stream);
  for ( auto &value : values_ )
  {
    *data = value.sec;
    ++data;
    *data = value.nsec;
    ++data;
  }
  return length;
}

//! ============== Duration Specialization ==============

template<>
ros::Duration ArrayMessage<ros::Duration>::operator[]( size_t index )
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    int32_t secs = *reinterpret_cast<const int32_t *>(stream_);
    int32_t nsecs = *reinterpret_cast<const int32_t *>(stream_ + 4);
    return { secs, nsecs };
  }
  return values_[index];
}

template<>
ros::Duration ArrayMessage<ros::Duration>::operator[]( size_t index ) const
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    int32_t secs = *reinterpret_cast<const int32_t *>(stream_);
    int32_t nsecs = *reinterpret_cast<const int32_t *>(stream_ + 4);
    return { secs, nsecs };
  }
  return values_[index];
}

template<>
size_t ArrayMessage<ros::Duration>::size() const
{
  return 2 * sizeof( int32_t ) * length_ + (fixed_length_ ? 0 : 4);
}

template<>
void ArrayMessage<ros::Duration>::detachFromStream()
{
  if ( !from_stream_ ) return;
  auto data = reinterpret_cast<const int32_t *>(stream_);
  values_.clear();
  for ( size_t i = 0; i < length_; ++i )
  {
    int32_t secs = *data;
    ++data;
    int32_t nsecs = *data;
    ++data;
    values_.emplace_back( secs, nsecs );
  }
  from_stream_ = false;
}

template<>
size_t ArrayMessage<ros::Duration>::writeToStream( uint8_t *stream ) const
{
  size_t length = size();
  size_t count = length;
  if ( !fixed_length_ )
  {
    *reinterpret_cast<uint32_t *>(stream) = length_;
    stream += 4;
    count -= 4;
  }
  if ( from_stream_ )
  {
    std::memcpy( stream, stream_, count );
    return length;
  }
  auto *data = reinterpret_cast<int32_t *>(stream);
  for ( auto &value : values_ )
  {
    *data = value.sec;
    ++data;
    *data = value.nsec;
    ++data;
  }
  return length;
}
}
