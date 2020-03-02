// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_VALUE_MESSAGE_H
#define ROS_BABEL_FISH_VALUE_MESSAGE_H

#include "ros_babel_fish/exceptions/babel_fish_exception.h"
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

  size_t _sizeInBytes() const override { return sizeof( T ); }

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

  static ValueMessage<T> *fromStream( const uint8_t *stream, size_t stream_length, size_t &bytes_read )
  {
    (void) stream_length; // For unused warning
    T val = *reinterpret_cast<const T *>(stream + bytes_read);
    bytes_read += sizeof( T );
    if ( bytes_read > stream_length )
      throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
    return new ValueMessage<T>( val );
  }

  ValueMessage<T> &operator=( const T &value )
  {
    setValue( value );
    return *this;
  }

  ValueMessage<T> &operator=( const ValueMessage<T> &other )
  {
    setValue( other.getValue());
    return *this;
  }

  Message *clone() const override
  {
    if ( isDetachedFromStream()) return new ValueMessage<T>( getValue());
    return new ValueMessage<T>( stream_ );
  }

protected:
  void assign( const Message &other ) override
  {
    if ( type != other.type()) throw BabelFishException( "Tried to assign incompatible message to ValueMessage!" );
    setValue( other.as<ValueMessage<T>>().getValue());
  }

  mutable T value_;
  mutable bool from_stream_;
};

// Bool specialization
template<>
bool ValueMessage<bool>::getValue() const;

template<>
size_t ValueMessage<bool>::_sizeInBytes() const;

template<>
ValueMessage<bool> *ValueMessage<bool>::fromStream( const uint8_t *stream, size_t stream_length, size_t &bytes_read );

template<>
size_t ValueMessage<bool>::writeToStream( uint8_t *stream ) const;

// Time specialization
template<>
ros::Time ValueMessage<ros::Time>::getValue() const;

template<>
size_t ValueMessage<ros::Time>::_sizeInBytes() const;

template<>
ValueMessage<ros::Time> *
ValueMessage<ros::Time>::fromStream( const uint8_t *stream, size_t stream_length, size_t &bytes_read );

template<>
size_t ValueMessage<ros::Time>::writeToStream( uint8_t *stream ) const;

// Duration specialization
template<>
ros::Duration ValueMessage<ros::Duration>::getValue() const;

template<>
size_t ValueMessage<ros::Duration>::_sizeInBytes() const;

template<>
ValueMessage<ros::Duration> *
ValueMessage<ros::Duration>::fromStream( const uint8_t *stream, size_t stream_length, size_t &bytes_read );

template<>
size_t ValueMessage<ros::Duration>::writeToStream( uint8_t *stream ) const;

// String specialization
template<>
std::string ValueMessage<std::string>::getValue() const;

template<>
size_t ValueMessage<std::string>::_sizeInBytes() const;

template<>
ValueMessage<std::string> *
ValueMessage<std::string>::fromStream( const uint8_t *stream, size_t stream_length, size_t &bytes_read );

template<>
size_t ValueMessage<std::string>::writeToStream( uint8_t *stream ) const;
} // ros_babel_fish

#endif //ROS_BABEL_FISH_VALUE_MESSAGE_H
