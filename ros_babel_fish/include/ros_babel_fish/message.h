// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_H
#define ROS_BABEL_FISH_MESSAGE_H


#include "ros_babel_fish/exceptions/babel_fish_exception.h"
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

/*!
 * Message representation used by BabelFish.
 * By default a translated message is lazy constructed. That means the data is not copied but a reference to the
 * message's internal buffer is stored. If data is changed or added, the data is copied.
 * Copying can be forced by calling Message::detachFromStream which will detach the message from the buffer at which
 * point the buffer can be safely destroyed.
 */
class Message
{
public:
  typedef std::shared_ptr<Message> Ptr;
  typedef std::shared_ptr<const Message> ConstPtr;

  Message( Message &other ) = delete;

  explicit Message( MessageType type, const uint8_t *stream = nullptr );

  virtual ~Message();

  MessageType type() const { return type_; }

  const uint8_t *_stream() const { return stream_; }

  /*!
   * Alias for _stream(). Removed in future release.
   */
  __attribute_deprecated__ const uint8_t *data() const { return _stream(); }

  /*!
   * @return Size of the message in bytes if serialized in ROS message binary format
   */
  virtual size_t _sizeInBytes() const = 0;

  /*!
   * Alias for _sizeInBytes(). Removed in future release.
   */
  __attribute_deprecated__ size_t size() const { return _sizeInBytes(); }

  /*!
   * @return Whether or not the message is detached from the stream. If false, the message is not fully copied and still relies on the translated message.
   */
  virtual bool isDetachedFromStream() const = 0;

  /**
   * Detaches the message from the underlying stream.
   * All values are copied to an internal storage and the stream is no longer needed.
   */
  virtual void detachFromStream() = 0;

  /**
   * Writes the message's content to the given stream using the ROS message binary format.
   * The stream has to be able to fit at least the number of bytes returned by _sizeInBytes.
   * @param stream The stream the message is written to
   * @return The number of bytes written
   */
  virtual size_t writeToStream( uint8_t *stream ) const = 0;

  /**
   * Convenience method to access the child with the given key  of a CompoundMessage.
   * @param key The name or path of the child
   * @return The child message accessed by the given key
   *
   * @throws BabelFishException If child access by key is not supported.
   */
  virtual Message &operator[]( const std::string &key );

  //!@copydoc Message::operator[](const std::string&)
  virtual const Message &operator[]( const std::string &key ) const;

  /**
   * Copies the content of other to this message.
   * @param other The other message
   * @return A reference to this message
   */
  Message &operator=( const Message &other )
  {
    assign( other );
    return *this;
  }

  /**
   * @defgroup Convenience methods for ValueMessage
   * @brief Will try to set the value of ValueMessage to the given value.
   * Incompatible types are checked at runtime. If the type of ValueMessage can not fit the passed value prints an error.
   *
   * @throws BabelFishException If bool is assigned to non-boolean ValueMessage or non-boolean value to bool ValueMessage
   * @throws BabelFishException If ros::Time / ros::Duration value is set to a different type of ValueMessage.
   * @{
   */
  Message &operator=( bool value );

  Message &operator=( uint8_t value );

  Message &operator=( uint16_t value );

  Message &operator=( uint32_t value );

  Message &operator=( uint64_t value );

  Message &operator=( int8_t value );

  Message &operator=( int16_t value );

  Message &operator=( int32_t value );

  Message &operator=( int64_t value );

  Message &operator=( float value );

  Message &operator=( double value );

  Message &operator=( const char *value )
  {
    *this = std::string( value );
    return *this;
  }

  Message &operator=( const std::string &value );

  Message &operator=( const ros::Time &value );

  Message &operator=( const ros::Duration &value );

  /**@}*/

  /**
   * Convenience method to obtain the content of a ValueMessage as the given type.
   * A type conversion is done if the type doesn't match exactly. If the target type can not fit the source type, a
   * warning is printed.
   *
   * @tparam T The type as which the value is retrieved
   * @return The value casted to the given type T
   *
   * @throws BabelFishException If the message is not a ValueMessage
   * @throws BabelFishException If the type of the ValueMessage can not be casted to a different type which is the case for bool, std::string, ros::Time and ros::Duration
   */
  template<typename T>
  T value() const
  {
    static_assert( sizeof( T ) == 0, "Please only call declared specializations!" );
  }


  /**
   * Clones the message including its content. The cloned message will be a full copy including for each part whether
   * it is detached from the stream or not.
   * @return A clone of the message.
   */
  virtual Message *clone() const = 0;

  /*!
   * Convenience method that casts the message to the given type.
   * Example:
   * @code
   * Message &msg = getMessage();
   * CompoundMessage &compound = msg.as<CompoundMessage>();
   * @endcode
   * @tparam T Target type
   * @return Message casted to the target type as reference
   *
   * @throws BabelFishException If the message can not be casted to the target type
   */
  template<typename T>
  T &as()
  {
    T *result = dynamic_cast<T *>(this);
    if ( result == nullptr ) throw BabelFishException( "Tried to cast message to incompatible type!" );
    return *result;
  }

  //! @copydoc Message::as()
  template<typename T>
  const T &as() const
  {
    const T *result = dynamic_cast<const T *>(this);
    if ( result == nullptr ) throw BabelFishException( "Tried to cast message to incompatible type!" );
    return *result;
  }

protected:
  virtual void assign( const Message &other ) = 0;

  const MessageType type_;
  const uint8_t *stream_; // TODO regard endianness
};


template<>
bool Message::value() const;

template<>
uint8_t Message::value() const;

template<>
uint16_t Message::value() const;

template<>
uint32_t Message::value() const;

template<>
uint64_t Message::value() const;

template<>
int8_t Message::value() const;

template<>
int16_t Message::value() const;

template<>
int32_t Message::value() const;

template<>
int64_t Message::value() const;

template<>
float Message::value() const;

template<>
double Message::value() const;

template<>
std::string Message::value() const;

template<>
ros::Time Message::value() const;

template<>
ros::Duration Message::value() const;

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
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::String, std::string );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Compound, Message );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Array, Message );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::None, void );
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
struct array_type<std::string>
{
  typedef std::string ReturnType;
  typedef std::string ConstReturnType;
  typedef const std::string &ArgumentType;
  typedef std::string StorageType;
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
