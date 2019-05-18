// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_H
#define ROS_BABEL_FISH_MESSAGE_H

#include <ros/time.h>

#include <memory>

namespace ros_babel_fish
{

namespace MessageTypes
{
enum MessageType : uint32_t
{
  None = 0x0000,
  Bool = 0x0001,
  UInt8 = 0x0002,
  UInt16 = 0x0004,
  UInt32 = 0x0008,
  UInt64 = 0x0010,
  Int8 = 0x0020,
  Int16 = 0x0040,
  Int32 = 0x0080,
  Int64 = 0x0100,
  Float32 = 0x0200,
  Float64 = 0x0400,
  String = 0x0800,
  Time = 0x1000,
  Duration = 0x2000,
  Compound = 0x4000,
  Array = 0x8000
};
}
typedef MessageTypes::MessageType MessageType;

class Message
{
public:
  typedef std::shared_ptr<Message> Ptr;
  typedef std::shared_ptr<const Message> ConstPtr;

  Message( const Message &other ) = delete;

  Message &operator=( const Message &other ) = delete;

  explicit Message( MessageType type, const uint8_t *stream = nullptr );

  virtual ~Message();

  uint32_t type() const { return type_; }

  const uint8_t *data() const { return stream_; }

  virtual size_t size() const = 0;

  virtual bool isDetachedFromStream() const = 0;

  /*!
   * Detaches the message from the underlying stream.
   * All values are copied to an internal storage and the stream is no longer needed.
   */
  virtual void detachFromStream() = 0;

  // TODO WriteToStream method documentation
  virtual size_t writeToStream( uint8_t *stream ) const = 0;

  /* Some convenience access functions */
  virtual Message &operator[]( const std::string &key );

  virtual const Message &operator[]( const std::string &key ) const;

  template<typename T>
  T &as()
  {
    T *result = dynamic_cast<T *>(this);
    if ( result == nullptr ) throw std::runtime_error( "Tried to cast message to incompatible type!" );
    return *result;
  }

  template<typename T>
  const T &as() const
  {
    const T *result = dynamic_cast<const T *>(this);
    if ( result == nullptr ) throw std::runtime_error( "Tried to cast message to incompatible type!" );
    return *result;
  }

protected:
  MessageType type_;
  const uint8_t *stream_; // TODO regard endianness
};

namespace message_type_traits
{
template<typename T>
struct message_type
{
  static constexpr MessageType value = MessageTypes::None;
};
#define DECLARE_MESSAGE_TYPE_FOR_TYPE( __message_type, __type )\
template<> struct message_type<__type> { static constexpr MessageType value = __message_type; }
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Bool, bool );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt8, uint8_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt16, uint16_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt32, uint32_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt64, uint64_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int8, int8_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int16, int16_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int32, int32_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int64, int64_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Float32, float );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Float64, double );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::String, std::string );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Time, ros::Time );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Duration, ros::Duration );
#undef DECLARE_MESSAGE_TYPE_FOR_TYPE

template<MessageType>
struct member_type
{
  typedef void value;
};
#define DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( __message_type, __type ) \
 template<> struct member_type<__message_type> { typedef __type value; }
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Bool, bool );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt8, uint8_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt16, uint16_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt32, uint32_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt64, uint64_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int8, int8_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int16, int16_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int32, int32_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int64, int64_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Float32, float );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Float64, double );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Time, ros::Time );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Duration, ros::Duration );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::String, Message );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Compound, Message );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Array, Message );
#undef DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE

template<MessageType>
struct value_type
{
  typedef void value;
};
#define DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( __message_type, __type ) \
 template<> struct value_type<__message_type> { typedef __type value; }
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Bool, bool );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt8, uint8_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt16, uint16_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt32, uint32_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt64, uint64_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int8, int8_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int16, int16_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int32, int32_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int64, int64_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Float32, float );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Float64, double );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Time, ros::Time );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Duration, ros::Duration );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::String, std::string );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Compound, Message );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Array, Message );
#undef DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE


template<typename T>
struct array_type
{
  typedef T ReturnType;
  typedef T ConstReturnType;
  typedef T ArgumentType;
  typedef T StorageType;
};
template<>
struct array_type<ros::Time>
{
  typedef ros::Time ReturnType;
  typedef ros::Time ConstReturnType;
  typedef const ros::Time &ArgumentType;
  typedef ros::Time StorageType;
};
template<>
struct array_type<ros::Duration>
{
  typedef ros::Duration ReturnType;
  typedef ros::Duration ConstReturnType;
  typedef const ros::Duration &ArgumentType;
  typedef ros::Duration StorageType;
};
template<>
struct array_type<Message>
{
  typedef Message &ReturnType;
  typedef const Message &ConstReturnType;
  typedef Message *ArgumentType;
  typedef Message *StorageType;
};
}
} // ros_babel_fish

#endif //ROS_BABEL_FISH_MESSAGE_H
