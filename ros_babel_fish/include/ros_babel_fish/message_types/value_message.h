// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_VALUE_MESSAGE_H
#define ROS_BABEL_FISH_VALUE_MESSAGE_H

#include "ros_babel_fish/message.h"

#include <ros/time.h>

namespace ros_babel_fish
{

template<typename T>
class ValueMessage : public Message
{
  static constexpr MessageType type = message_type_traits::message_type<T>::value;
  static_assert( type != MessageTypes::None, "Invalid type parameter for ValueMessage!" );
public:
  explicit ValueMessage( T value = T())
    : Message( type ), value_( value ), from_stream_( false ) { }

  explicit ValueMessage( const uint8_t *stream )
    : Message( type, stream ), value_( T()), from_stream_( stream != nullptr ) { }

  T getValue() const
  {
    if ( from_stream_ ) return *reinterpret_cast<const T *>(stream_);
    return value_;
  }

  void setValue( T value )
  {
    value_ = value;
    from_stream_ = false;
  }

  size_t size() const override { return sizeof( T ); }

  bool isDetachedFromStream() const override
  {
    return !from_stream_;
  }

  void detachFromStream() override
  {
    if ( !from_stream_ ) return;
    value_ = getValue();
    from_stream_ = false;
  }

  size_t writeToStream( uint8_t *stream ) const override
  {
    *reinterpret_cast<T *>(stream) = getValue();
    return sizeof( T );
  }

  static ValueMessage<T> *fromData( const uint8_t *data, size_t &bytes_read )
  {
    bytes_read = sizeof( T );
    return new ValueMessage<T>( *reinterpret_cast<const T *>(data));
  }

protected:
  mutable T value_;
  mutable bool from_stream_;
};

// Bool specialization
template<>
bool ValueMessage<bool>::getValue() const;

template<>
size_t ValueMessage<bool>::size() const;

template<>
ValueMessage<bool> *ValueMessage<bool>::fromData( const uint8_t *data, size_t &bytes_read );

template<>
size_t ValueMessage<bool>::writeToStream( uint8_t *stream ) const;

// Time specialization
template<>
ros::Time ValueMessage<ros::Time>::getValue() const;

template<>
size_t ValueMessage<ros::Time>::size() const;

template<>
ValueMessage<ros::Time> *ValueMessage<ros::Time>::fromData( const uint8_t *data, size_t &bytes_read );

template<>
size_t ValueMessage<ros::Time>::writeToStream( uint8_t *stream ) const;

// Duration specialization
template<>
ros::Duration ValueMessage<ros::Duration>::getValue() const;

template<>
size_t ValueMessage<ros::Duration>::size() const;

template<>
ValueMessage<ros::Duration> *ValueMessage<ros::Duration>::fromData( const uint8_t *data, size_t &bytes_read );

template<>
size_t ValueMessage<ros::Duration>::writeToStream( uint8_t *stream ) const;

// String specialization
template<>
std::string ValueMessage<std::string>::getValue() const;

template<>
size_t ValueMessage<std::string>::size() const;

template<>
ValueMessage<std::string> *ValueMessage<std::string>::fromData( const uint8_t *data, size_t &bytes_read );

template<>
size_t ValueMessage<std::string>::writeToStream( uint8_t *stream ) const;
} // ros_babel_fish

#endif //ROS_BABEL_FISH_VALUE_MESSAGE_H
