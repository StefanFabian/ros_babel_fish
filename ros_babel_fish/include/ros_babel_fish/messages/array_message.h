// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_ARRAY_MESSAGE_H
#define ROS_BABEL_FISH_ARRAY_MESSAGE_H

#include "ros_babel_fish/exceptions.h"
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
  void setItem( size_t index, ArgumentType value )
  {
    if ( index >= values_.size())
      throw BabelFishException( "Index in setItem was out of bounds! Maybe you meant addItem?" );
    if ( from_stream_ ) detachFromStream();
    values_[index] = value;
  }

  void addItem( ArgumentType value )
  {
    if ( fixed_length_ )
    {
      throw BabelFishException( "Can not add items to a fixed size array!" );
    }
    if ( from_stream_ ) detachFromStream();
    values_.push_back( value );
    ++length_;
  }

  size_t size() const override
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
size_t ArrayMessage<Message>::size() const;

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
public:
  explicit CompoundArrayMessage( std::string datatype, size_t length = 0, bool fixed_length = false,
                                 const uint8_t *stream = nullptr )
    : ArrayMessage<Message>( MessageTypes::Compound, length, fixed_length, stream ), datatype_( std::move( datatype ))
  {
  }

  const std::string &elementDataType() const { return datatype_; }

  Message *clone() const override;

private:
  std::string datatype_;
};

//! Specialization for Bool
template<>
bool ArrayMessage<bool>::operator[]( size_t index );

template<>
bool ArrayMessage<bool>::operator[]( size_t index ) const;

template<>
size_t ArrayMessage<bool>::size() const;

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
size_t ArrayMessage<std::string>::size() const;

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
size_t ArrayMessage<ros::Time>::size() const;

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
size_t ArrayMessage<ros::Duration>::size() const;

template<>
void ArrayMessage<ros::Duration>::detachFromStream();

template<>
size_t ArrayMessage<ros::Duration>::writeToStream( uint8_t *stream ) const;
} // ros_babel_fish

#endif //ROS_BABEL_FISH_ARRAY_MESSAGE_H
