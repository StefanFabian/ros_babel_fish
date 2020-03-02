// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_ARRAY_MESSAGE_H
#define ROS_BABEL_FISH_ARRAY_MESSAGE_H

#include "ros_babel_fish/generation/message_template.h"
#include "ros_babel_fish/exceptions/babel_fish_exception.h"
#include "ros_babel_fish/message.h"

#include <ros/time.h>

namespace ros_babel_fish
{

class ArrayMessageBase : public Message
{
public:
  ArrayMessageBase( MessageType element_type, size_t length, bool fixed_length, const uint8_t *stream = nullptr )
    : Message( MessageTypes::Array, stream ), element_type_( element_type ), length_( length )
      , fixed_length_( fixed_length ) { }

  MessageType elementType() const { return element_type_; }

  bool isFixedSize() const { return fixed_length_; }

  size_t length() const { return length_; }

protected:
  MessageType element_type_;
  size_t length_;
  bool fixed_length_;
};

template<typename T>
class ArrayMessage : public ArrayMessageBase
{
protected:
  typedef typename message_type_traits::array_type<T>::ReturnType ReturnType;
  typedef typename message_type_traits::array_type<T>::ConstReturnType ConstReturnType;
  typedef typename message_type_traits::array_type<T>::ArgumentType ArgumentType;
  typedef typename message_type_traits::array_type<T>::StorageType StorageType;

  ArrayMessage( MessageType element_type, size_t length, bool fixed_length, const uint8_t *stream, bool )
    : ArrayMessageBase( element_type, length, fixed_length, stream )
      , values_( stream == nullptr ? length : 0 ), from_stream_( stream != nullptr ) { }

public:
  template<typename T1 = T, typename std::enable_if<std::is_same<T1, Message>::value, int>::type = 0>
  explicit ArrayMessage( MessageType element_type, size_t length = 0, bool fixed_length = false,
                         const uint8_t *stream = nullptr )
    : ArrayMessage( element_type, length, fixed_length, stream, true )
  {
  }

  template<typename T1 = T, typename std::enable_if<!std::is_same<T1, Message>::value, int>::type = 0>
  explicit ArrayMessage( size_t length = 0, bool fixed_length = false, const uint8_t *stream = nullptr )
    : ArrayMessage( message_type_traits::message_type<T>::value, length, fixed_length, stream, true )
  {
  }

  ~ArrayMessage() override { }

  static ArrayMessage<T> *fromStream( ssize_t length, const uint8_t *stream, size_t stream_length, size_t &bytes_read )
  {
    (void) stream_length; // For unused warning
    bool fixed_length = length >= 0;
    stream += bytes_read;
    if ( !fixed_length )
    {
      length = *reinterpret_cast<const uint32_t *>( stream );
      stream += sizeof( uint32_t );
      bytes_read += sizeof( uint32_t );
    }
    bytes_read += sizeof( T ) * length;
    if ( bytes_read > stream_length )
      throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
    return new ArrayMessage<T>( length, fixed_length, stream );
  }

  ReturnType operator[]( size_t index )
  {
    if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
    if ( from_stream_ ) return *(reinterpret_cast<const T *>(stream_) + index);
    return values_[index];
  }

  ConstReturnType operator[]( size_t index ) const
  {
    if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
    if ( from_stream_ ) return *(reinterpret_cast<const T *>(stream_) + index);
    return values_[index];
  }

  ReturnType at( size_t index ) { return operator[]( index ); }

  ConstReturnType at( size_t index ) const { return operator[]( index ); }

  /*!
   * @param index The index at which the array element is set/overwritten
   * @param value The value with which the array element is overwritten, has to be the same as the element type.
   */
  void assign( size_t index, ArgumentType value )
  {
    if ( index >= length_ )
      throw BabelFishException( "Index in setItem was out of bounds! Maybe you meant push_back?" );
    if ( from_stream_ ) detachFromStream();
    values_[index] = value;
  }

  /*!
   * Alias for assign
   */
  void replace( size_t index, ArgumentType value ) { assign( index, value ); }

  /*!
   * Alias for assign
   * Deprecated, will be removed in a future release
   */
  __attribute_deprecated__ void setItem( size_t index, ArgumentType value ) { assign( index, value ); }

  void push_back( ArgumentType value )
  {
    if ( fixed_length_ )
    {
      throw BabelFishException( "Can not add items to a fixed size array!" );
    }
    if ( from_stream_ ) detachFromStream();
    values_.push_back( value );
    ++length_;
  }

  /*!
   * Alias for push_back
   */
  void append( ArgumentType value ) { push_back( value ); }

  /*!
   * Alias for push_back
   * Deprecated, will be removed in a future release
   */
  __attribute_deprecated__ void addItem( ArgumentType value ) { push_back( value ); }

  size_t _sizeInBytes() const override
  {
    return sizeof( T ) * length_ + (fixed_length_ ? 0 : 4);
  }

  void reserve( size_t length )
  {
    values_.reserve( length );
  }

  bool isDetachedFromStream() const override { return !from_stream_; }

  void detachFromStream() override
  {
    if ( !from_stream_ ) return;
    auto data = reinterpret_cast<const T *>(stream_);
    values_.clear();
    values_.reserve( length_ );
    for ( size_t i = 0; i < length_; ++i )
    {
      values_.push_back( *data );
      ++data;
    }
    from_stream_ = false;
  }

  size_t writeToStream( uint8_t *stream ) const override
  {
    size_t length = _sizeInBytes();
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
    std::memcpy( stream, values_.data(), count );
    return length;
  }

  ArrayMessage<T> &operator=( const ArrayMessage<T> &other )
  {
    from_stream_ = other.from_stream_;
    stream_ = other.stream_;
    length_ = other.length_;
    fixed_length_ = other.fixed_length_;
    values_.clear();
    values_ = other.values_;
    return *this;
  }

  Message *clone() const override
  {
    auto result = new ArrayMessage<T>( elementType(), length(), isFixedSize(), stream_, true );
    result->from_stream_ = from_stream_;
    result->values_ = values_;
    return result;
  }

protected:
  void assign( const Message &other ) override
  {
    auto o = dynamic_cast<const ArrayMessage<T> *>(&other);
    if ( o == nullptr ) throw BabelFishException( "Tried to assign incompatible Message type to ArrayMessage!" );
    *this = *o;
  }

protected:
  std::vector<StorageType> values_;
  bool from_stream_;
};


//! Specialization for Message
template<>
ArrayMessage<Message>::ArrayMessage( MessageType element_type, size_t length, bool fixed_length,
                                     const uint8_t *stream, bool );

template<>
ArrayMessage<Message>::~ArrayMessage();

template<>
Message &ArrayMessage<Message>::operator[]( size_t index );

template<>
const Message &ArrayMessage<Message>::operator[]( size_t index ) const;

template<>
void ArrayMessage<Message>::assign( size_t index, Message *value );

template<>
size_t ArrayMessage<Message>::_sizeInBytes() const;

template<>
size_t ArrayMessage<Message>::writeToStream( uint8_t *stream ) const;

template<>
inline void ArrayMessage<Message>::detachFromStream()
{
  /* So compiler won't complain. This specialization can not be from stream anyway. */
}

template<>
ArrayMessage<Message> &ArrayMessage<Message>::operator=( const ArrayMessage<Message> &other );

template<>
Message *ArrayMessage<Message>::clone() const;

//! Specialization for CompoundMessage
class CompoundArrayMessage : public ArrayMessage<Message>
{
  explicit CompoundArrayMessage( MessageTemplate::ConstPtr msg_template, size_t length, bool fixed_length,
                                 const uint8_t *stream );

public:
  /*!
   * Creates a compound array, i.e., an array of compound messages in contrast to arrays of primitives such as int, bool etc.
   * If length != 0, the array is initialized with the given number of empty messages created according to the given
   * MessageTemplate.
   *
   * @param msg_template The template for the CompoundMessage
   * @param length The length of the array.
   * @param fixed_length Whether the array has fixed length or elements can be added / removed dynamically.
   */
  explicit CompoundArrayMessage( MessageTemplate::ConstPtr msg_template, size_t length = 0, bool fixed_length = false );

  static CompoundArrayMessage *fromStream( ssize_t length, MessageTemplate::ConstPtr msg_template,
                                           const uint8_t *stream, size_t stream_length, size_t &bytes_read );

  const std::string &elementDataType() const { return msg_template_->compound.datatype; }

  const MessageTemplate::ConstPtr &elementTemplate() { return msg_template_; }

  Message &appendEmpty();

  Message *clone() const override;

private:
  MessageTemplate::ConstPtr msg_template_;
};

//! Specialization for Bool
template<>
bool ArrayMessage<bool>::operator[]( size_t index );

template<>
bool ArrayMessage<bool>::operator[]( size_t index ) const;

template<>
ArrayMessage<bool> *ArrayMessage<bool>::fromStream( ssize_t length, const uint8_t *stream, size_t stream_length,
                                                    size_t &bytes_read );

template<>
size_t ArrayMessage<bool>::_sizeInBytes() const;

template<>
void ArrayMessage<bool>::detachFromStream();

template<>
size_t ArrayMessage<bool>::writeToStream( uint8_t *stream ) const;

//! Specialization for String
template<>
std::string ArrayMessage<std::string>::operator[]( size_t index );

template<>
std::string ArrayMessage<std::string>::operator[]( size_t index ) const;

template<>
ArrayMessage<std::string> *ArrayMessage<std::string>::fromStream( ssize_t length, const uint8_t *stream,
                                                                  size_t stream_length, size_t &bytes_read );

template<>
size_t ArrayMessage<std::string>::_sizeInBytes() const;

template<>
void ArrayMessage<std::string>::detachFromStream();

template<>
size_t ArrayMessage<std::string>::writeToStream( uint8_t *stream ) const;

//! Specialization for Time
template<>
ros::Time ArrayMessage<ros::Time>::operator[]( size_t index );

template<>
ros::Time ArrayMessage<ros::Time>::operator[]( size_t index ) const;

template<>
ArrayMessage<ros::Time> *ArrayMessage<ros::Time>::fromStream( ssize_t length, const uint8_t *stream,
                                                              size_t stream_length, size_t &bytes_read );

template<>
size_t ArrayMessage<ros::Time>::_sizeInBytes() const;

template<>
void ArrayMessage<ros::Time>::detachFromStream();

template<>
size_t ArrayMessage<ros::Time>::writeToStream( uint8_t *stream ) const;

//! Specialization for Duration

template<>
ros::Duration ArrayMessage<ros::Duration>::operator[]( size_t index );

template<>
ros::Duration ArrayMessage<ros::Duration>::operator[]( size_t index ) const;

template<>
ArrayMessage<ros::Duration> *ArrayMessage<ros::Duration>::fromStream( ssize_t length, const uint8_t *stream,
                                                                      size_t stream_length, size_t &bytes_read );

template<>
size_t ArrayMessage<ros::Duration>::_sizeInBytes() const;

template<>
void ArrayMessage<ros::Duration>::detachFromStream();

template<>
size_t ArrayMessage<ros::Duration>::writeToStream( uint8_t *stream ) const;
} // ros_babel_fish

#endif //ROS_BABEL_FISH_ARRAY_MESSAGE_H
