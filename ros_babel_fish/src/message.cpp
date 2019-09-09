// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/messages/compound_message.h"
#include "ros_babel_fish/messages/value_message.h"
#include "ros_babel_fish/babel_fish.h"
#include "ros_babel_fish/message.h"

#include <limits>

namespace ros_babel_fish
{

Message::Message( MessageType type, const uint8_t *stream ) : type_( type ), stream_( stream ) { }

Message &Message::operator[]( const std::string & )
{
  throw BabelFishException(
    "Tried to access child message on message object that does not support child access by key." );
}

const Message &Message::operator[]( const std::string & ) const
{
  throw BabelFishException(
    "Tried to access child message on message object that does not support child access by key." );
}

Message::~Message() = default;

namespace
{

template<typename T, typename U>
typename std::enable_if<std::numeric_limits<T>::is_signed != std::numeric_limits<U>::is_signed, bool>::type
inBounds() { return false; }

template<typename T, typename U>
typename std::enable_if<std::numeric_limits<T>::is_signed == std::numeric_limits<U>::is_signed, bool>::type inBounds()
{
  return std::numeric_limits<U>::min() <= std::numeric_limits<T>::min() &&
         std::numeric_limits<T>::max() <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<std::is_integral<U>::value, bool>::type isCompatible()
{
  if ( std::is_same<T, U>::value ) return true;
  if ( std::is_floating_point<T>::value ) return false;
  if ( std::numeric_limits<T>::is_signed && !std::numeric_limits<U>::is_signed )
    return false;
  return inBounds<T, U>();
}

template<typename T, typename U>
typename std::enable_if<std::is_floating_point<U>::value, bool>::type isCompatible()
{
  return std::is_same<T, U>::value || !std::is_same<T, double>::value;
}

template<>
bool isCompatible<float, double>() { return true; }

template<typename T, typename U>
typename std::enable_if<std::is_floating_point<T>::value, bool>::type
inBounds( const T & )
{
  return false;
}


// is_integral is necessary to avoid a CLang tidy warning
template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value && std::numeric_limits<T>::is_signed && !std::numeric_limits<U>::is_signed, bool>::type
inBounds( const T &val )
{
  return val >= 0 && static_cast<typename std::make_unsigned<T>::type >(val) <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value && !std::numeric_limits<T>::is_signed && std::numeric_limits<U>::is_signed, bool>::type
inBounds( const T &val )
{
  return val <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value && std::numeric_limits<T>::is_signed == std::numeric_limits<U>::is_signed, bool>::type
inBounds( const T &val )
{
  return std::numeric_limits<U>::min() <= val &&
         val <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<std::is_integral<U>::value, bool>::type isCompatible( const T &val )
{
  if ( std::is_same<T, U>::value ) return true;
  if ( std::is_floating_point<T>::value ) return false;
  return inBounds<T, U>( val );
}

template<typename T, typename U>
typename std::enable_if<std::is_floating_point<U>::value, bool>::type isCompatible( const T & )
{
  return std::is_integral<T>::value || sizeof( T ) <= sizeof( U );
}

template<typename T, typename U>
void assignValue( Message *m, const T &value )
{
  using namespace message_type_traits;
  if ( m->type() != message_type<T>::value && !isCompatible<T, U>( value ))
    throw BabelFishException(
      "Value does not fit into value message! Make sure you're using the correct type or at least stay within the range of values for the message type!" );
#if RBF_WARN_ON_INCOMPATIBLE_TYPE
  if ( m->type() != message_type<T>::value && !isCompatible<T, U>())
    ROS_WARN_ONCE_NAMED( "RosBabelFish",
                         "Assigned value fits but the type of the assignment can not be converted without loss of information in some cases! This message is printed only once!" );
#endif
  m->as<ValueMessage<U>>().setValue( static_cast<U>(value));
}

template<typename T>
void assignToValueMessage( Message *m, const T &value )
{
  using namespace message_type_traits;
  switch ( m->type())
  {
    case MessageTypes::Bool:
      throw BabelFishException( "Can not assign non-boolean value to a boolean ValueMessage!" );
    case MessageTypes::UInt8:
      assignValue<T, typename value_type<MessageTypes::UInt8>::value>( m, value );
      break;
    case MessageTypes::UInt16:
      assignValue<T, typename value_type<MessageTypes::UInt16>::value>( m, value );
      break;
    case MessageTypes::UInt32:
      assignValue<T, typename value_type<MessageTypes::UInt32>::value>( m, value );
      break;
    case MessageTypes::UInt64:
      assignValue<T, typename value_type<MessageTypes::UInt64>::value>( m, value );
      break;
    case MessageTypes::Int8:
      assignValue<T, typename value_type<MessageTypes::Int8>::value>( m, value );
      break;
    case MessageTypes::Int16:
      assignValue<T, typename value_type<MessageTypes::Int16>::value>( m, value );
      break;
    case MessageTypes::Int32:
      assignValue<T, typename value_type<MessageTypes::Int32>::value>( m, value );
      break;
    case MessageTypes::Int64:
      assignValue<T, typename value_type<MessageTypes::Int64>::value>( m, value );
      break;
    case MessageTypes::Float32:
      assignValue<T, typename value_type<MessageTypes::Float32>::value>( m, value );
      break;
    case MessageTypes::Float64:
      assignValue<T, typename value_type<MessageTypes::Float64>::value>( m, value );
      break;
    case MessageTypes::Duration:
      throw BabelFishException( "Can not assign non-duration value to a duration ValueMessage!" );
    case MessageTypes::Time:
      throw BabelFishException( "Can not assign non-time value to a time ValueMessage!" );
    case MessageTypes::String:
      throw BabelFishException( "Can not assign non-string value to a string ValueMessage!" );
    default:
      throw BabelFishException( "Tried to assign value to Message that is not a ValueMessage!" );
  }
}
}

Message &Message::operator=( bool value )
{
  if ( type() != MessageTypes::Bool )
    throw BabelFishException( "Can not assign a boolean to a non-boolean ValueMessage!" );
  as<ValueMessage<bool>>() = value;
  return *this;
}

Message &Message::operator=( uint8_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( uint16_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( uint32_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( uint64_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( int8_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( int16_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( int32_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( int64_t value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( float value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( double value )
{
  assignToValueMessage( this, value );
  return *this;
}

Message &Message::operator=( const std::string &value )
{
  if ( type() != MessageTypes::String )
    throw BabelFishException( "Can not assign a string to a non-string ValueMessage!" );
  as<ValueMessage<std::string>>() = value;
  return *this;
}

Message &Message::operator=( const ros::Time &value )
{
  if ( type() != MessageTypes::Time )
    throw BabelFishException( "Can not assign ros::Time to Message that is not ValueMessage<ros::Time>!" );
  as<ValueMessage<ros::Time>>() = value;
  return *this;
}

Message &Message::operator=( const ros::Duration &value )
{
  if ( type() != MessageTypes::Duration )
    throw BabelFishException( "Can not assign ros::Duration to Message that is not ValueMessage<ros::Duration>!" );
  as<ValueMessage<ros::Duration>>() = value;
  return *this;
}


namespace
{
template<typename T, typename U>
U obtainValue( const Message *m )
{
  using namespace message_type_traits;
  T val = m->as<ValueMessage<T>>().getValue();
  if ( m->type() != message_type<U>::value && !isCompatible<T, U>( val ))
    throw BabelFishException( "Value does not fit into casted type!" );
#if RBF_WARN_ON_INCOMPATIBLE_TYPE
  if ( m->type() != message_type<T>::value && !isCompatible<T, U>())
    ROS_WARN_ONCE_NAMED( "RosBabelFish",
                         "Value fits into casted type but it is smaller than the message type which may lead to catastrophic failure in the future! This message is printed only once!" );
#endif
  return static_cast<U>(val);
}

template<typename T>
T obtainValueAsType( const Message *m )
{
  using namespace message_type_traits;
  switch ( m->type())
  {
    case MessageTypes::UInt8:
      return obtainValue<typename value_type<MessageTypes::UInt8>::value, T>( m );
    case MessageTypes::UInt16:
      return obtainValue<typename value_type<MessageTypes::UInt16>::value, T>( m );
    case MessageTypes::UInt32:
      return obtainValue<typename value_type<MessageTypes::UInt32>::value, T>( m );
    case MessageTypes::UInt64:
      return obtainValue<typename value_type<MessageTypes::UInt64>::value, T>( m );
    case MessageTypes::Int8:
      return obtainValue<typename value_type<MessageTypes::Int8>::value, T>( m );
    case MessageTypes::Int16:
      return obtainValue<typename value_type<MessageTypes::Int16>::value, T>( m );
    case MessageTypes::Int32:
      return obtainValue<typename value_type<MessageTypes::Int32>::value, T>( m );
    case MessageTypes::Int64:
      return obtainValue<typename value_type<MessageTypes::Int64>::value, T>( m );
    case MessageTypes::Float32:
      return obtainValue<typename value_type<MessageTypes::Float32>::value, T>( m );
    case MessageTypes::Float64:
      return obtainValue<typename value_type<MessageTypes::Float64>::value, T>( m );
    default:
      throw BabelFishException( "Tried to retrieve content of ValueMessage as incompatible type!" );
  }
}
}

template<>
bool Message::value() const
{
  if ( type() != MessageTypes::Bool )
    throw BabelFishException( "Can not return value of non-boolean ValueMessage as boolean!" );
  return as<ValueMessage<bool>>().getValue();
}

template<>
uint8_t Message::value() const { return obtainValueAsType<uint8_t>( this ); }

template<>
uint16_t Message::value() const { return obtainValueAsType<uint16_t>( this ); }

template<>
uint32_t Message::value() const { return obtainValueAsType<uint32_t>( this ); }

template<>
uint64_t Message::value() const { return obtainValueAsType<uint64_t>( this ); }

template<>
int8_t Message::value() const { return obtainValueAsType<int8_t>( this ); }

template<>
int16_t Message::value() const { return obtainValueAsType<int16_t>( this ); }

template<>
int32_t Message::value() const { return obtainValueAsType<int32_t>( this ); }

template<>
int64_t Message::value() const { return obtainValueAsType<int64_t>( this ); }

template<>
float Message::value() const { return obtainValueAsType<float>( this ); }

template<>
double Message::value() const { return obtainValueAsType<double>( this ); }

template<>
std::string Message::value() const
{
  if ( type() != MessageTypes::String )
    throw BabelFishException( "Can not return value of non-string ValueMessage as string!" );
  return as<ValueMessage<std::string>>().getValue();
}

template<>
ros::Time Message::value() const
{
  if ( type() != MessageTypes::Time )
    throw BabelFishException( "Can not return value of non-time ValueMessage as ros::Time!" );
  return as<ValueMessage<ros::Time>>().getValue();
}

template<>
ros::Duration Message::value() const
{
  if ( type() != MessageTypes::Duration )
    throw BabelFishException( "Can not return value of non-duration ValueMessage as ros::Duration!" );
  return as<ValueMessage<ros::Duration>>().getValue();
}
}
