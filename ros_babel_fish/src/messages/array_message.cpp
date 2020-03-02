// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/messages/array_message.h"
#include "ros_babel_fish/messages/compound_message.h"

namespace ros_babel_fish
{

//! ===================================================
//! ============== Message Specialization =============
//! ===================================================

template<>
ArrayMessage<Message>::ArrayMessage( MessageType element_type, size_t length, bool fixed_length, const uint8_t *stream,
                                     bool )
  : ArrayMessageBase( element_type, length, fixed_length, stream ), values_( 0 ), from_stream_( false ) { }

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
void ArrayMessage<Message>::assign( size_t index, Message *value )
{
  if ( index >= length_ )
    throw BabelFishException( "Index in setItem was out of bounds! Maybe you meant push_back?" );
  delete values_[index];
  values_[index] = value;
}

template<>
size_t ArrayMessage<Message>::_sizeInBytes() const
{
  size_t result = (fixed_length_ ? 0 : 4);
  for ( auto &value : values_ )
  {
    result += value->_sizeInBytes();
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

template<>
ArrayMessage<Message> &ArrayMessage<Message>::operator=( const ArrayMessage<Message> &other )
{
  if ( type() != other.type() ||
       (elementType() == MessageTypes::Compound &&
        as<CompoundArrayMessage>().elementDataType() != other.as<CompoundArrayMessage>().elementDataType()))
    throw BabelFishException( "Can not assign incompatible ArrayMessage! They need to have exactly the same type!" );
  for ( auto &entry : values_ )
  {
    delete entry;
  }
  values_.clear();
  values_.reserve( other._sizeInBytes());
  std::transform( other.values_.begin(), other.values_.end(), std::back_inserter( values_ ),
                  []( Message *m ) { return m->clone(); } );
  length_ = other.length_;
  fixed_length_ = other.fixed_length_;
  stream_ = other.stream_;
  return *this;
}

template<>
Message *ArrayMessage<Message>::clone() const
{
  auto result = new ArrayMessage<Message>( elementType(), length(), isFixedSize(), stream_ );
  result->values_.clear();
  std::transform( values_.begin(), values_.end(), std::back_inserter( result->values_ ),
                  []( Message *m ) { return m->clone(); } );
  return result;
}

//! ===================================================
//! =============== Bool Specialization ===============
//! ===================================================

template<>
bool ArrayMessage<bool>::operator[]( size_t index )
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    return *(stream_ + index) != 0;
  }
  return values_[index];
}

template<>
bool ArrayMessage<bool>::operator[]( size_t index ) const
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    return *(stream_ + index) != 0;
  }
  return values_[index];
}

template<>
ArrayMessage<bool> *ArrayMessage<bool>::fromStream( ssize_t length, const uint8_t *stream, size_t stream_length,
                                                    size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  bool fixed_length = length >= 0;
  stream += bytes_read;
  if ( !fixed_length )
  {
    length = *reinterpret_cast<const uint32_t *>(stream);
    stream += sizeof( uint32_t );
    bytes_read += sizeof( uint32_t );
  }
  bytes_read += sizeof( uint8_t ) * length;
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
  return new ArrayMessage<bool>( length, fixed_length, stream );
}

template<>
size_t ArrayMessage<bool>::_sizeInBytes() const
{
  return sizeof( uint8_t ) * length_ + (fixed_length_ ? 0 : 4);
}

template<>
void ArrayMessage<bool>::detachFromStream()
{
  if ( !from_stream_ ) return;
  const uint8_t *data = stream_;
  values_.clear();
  values_.reserve( length_ );
  for ( size_t i = 0; i < length_; ++i )
  {
    values_.push_back( *data != 0 );
    ++data;
  }
  from_stream_ = false;
}

template<>
size_t ArrayMessage<bool>::writeToStream( uint8_t *stream ) const
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
  for ( size_t i = 0; i < length_; ++i )
  {
    *stream = static_cast<uint8_t>(values_[i] ? 1 : 0);
    ++stream;
  }
  return length;
}

//! ===================================================
//! ============== String Specialization ==============
//! ===================================================

template<>
std::string ArrayMessage<std::string>::operator[]( size_t index )
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    size_t offset = 0;
    for ( size_t i = 0; i < index; ++i )
    {
      offset += *reinterpret_cast<const uint32_t *>(stream_ + offset) + sizeof( uint32_t );
    }
    uint32_t len = *reinterpret_cast<const uint32_t *>(stream_ + offset);
    offset += sizeof( uint32_t );
    return std::string( reinterpret_cast<const char *>(stream_ + offset), len );
  }
  return values_[index];
}

template<>
std::string ArrayMessage<std::string>::operator[]( size_t index ) const
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    size_t offset = 0;
    for ( size_t i = 0; i < index; ++i )
    {
      offset += *reinterpret_cast<const uint32_t *>(stream_ + offset) + sizeof( uint32_t );
    }
    uint32_t len = *reinterpret_cast<const uint32_t *>(stream_ + offset);
    offset += sizeof( uint32_t );
    return std::string( reinterpret_cast<const char *>(stream_ + offset), len );
  }
  return values_[index];
}

template<>
ArrayMessage<std::string> *ArrayMessage<std::string>::fromStream( ssize_t length, const uint8_t *stream,
                                                                  size_t stream_length, size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  bool fixed_length = length >= 0;
  stream += bytes_read;
  if ( !fixed_length )
  {
    length = *reinterpret_cast<const uint32_t *>(stream);
    stream += sizeof( uint32_t );
    bytes_read += sizeof( uint32_t );
  }
  size_t offset = 0;
  for ( ssize_t i = 0; i < length; ++i )
  {
    offset += *reinterpret_cast<const uint32_t *>(stream + offset) + sizeof( uint32_t );
    if ( bytes_read + offset > stream_length )
      throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
  }
  bytes_read += offset;
  return new ArrayMessage<std::string>( length, fixed_length, stream );
}

template<>
size_t ArrayMessage<std::string>::_sizeInBytes() const
{
  size_t size = fixed_length_ ? 0 : 4;
  if ( from_stream_ )
  {
    size_t offset = 0;
    for ( size_t i = 0; i < length_; ++i )
    {
      offset += *reinterpret_cast<const uint32_t *>(stream_ + offset) + sizeof( uint32_t );
    }
    return size + offset;
  }
  for ( const auto &value : values_ )
  {
    size += value.length() + sizeof( uint32_t );
  }
  return size;
}

template<>
void ArrayMessage<std::string>::detachFromStream()
{
  if ( !from_stream_ ) return;
  auto data = stream_;
  values_.clear();
  for ( size_t i = 0; i < length_; ++i )
  {
    uint32_t len = *reinterpret_cast<const uint32_t *>(data);
    data += sizeof( uint32_t );
    values_.emplace_back( reinterpret_cast<const char *>(data), len );
    data += len;
  }
  from_stream_ = false;
}

template<>
size_t ArrayMessage<std::string>::writeToStream( uint8_t *stream ) const
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
  for ( auto &value : values_ )
  {
    *reinterpret_cast<uint32_t *>(stream) = value.length();
    stream += sizeof( uint32_t );
    std::memcpy( stream, value.data(), value.length());
    stream += value.length();
  }
  return length;
}

//! ===================================================
//! ================ Time Specialization ==============
//! ===================================================

template<>
ros::Time ArrayMessage<ros::Time>::operator[]( size_t index )
{
  if ( index >= length_ ) throw std::runtime_error( "Index out of message array bounds!" );
  if ( from_stream_ )
  {
    size_t offset = index * 8;
    uint32_t secs = *reinterpret_cast<const uint32_t *>(stream_ + offset);
    uint32_t nsecs = *reinterpret_cast<const uint32_t *>(stream_ + offset + 4);
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
    size_t offset = index * 8;
    uint32_t secs = *reinterpret_cast<const uint32_t *>(stream_ + offset);
    uint32_t nsecs = *reinterpret_cast<const uint32_t *>(stream_ + offset + 4);
    return { secs, nsecs };
  }
  return values_[index];
}

template<>
ArrayMessage<ros::Time> *ArrayMessage<ros::Time>::fromStream( ssize_t length, const uint8_t *stream,
                                                              size_t stream_length, size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  bool fixed_length = length >= 0;
  stream += bytes_read;
  if ( !fixed_length )
  {
    length = *reinterpret_cast<const uint32_t *>(stream);
    stream += sizeof( uint32_t );
    bytes_read += sizeof( uint32_t );
  }
  bytes_read += 2 * sizeof( uint32_t ) * length;
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
  return new ArrayMessage<ros::Time>( length, fixed_length, stream );
}

template<>
size_t ArrayMessage<ros::Time>::_sizeInBytes() const
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
    size_t offset = index * 8;
    int32_t secs = *reinterpret_cast<const int32_t *>(stream_ + offset);
    int32_t nsecs = *reinterpret_cast<const int32_t *>(stream_ + offset + 4);
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
    size_t offset = index * 8;
    int32_t secs = *reinterpret_cast<const int32_t *>(stream_ + offset);
    int32_t nsecs = *reinterpret_cast<const int32_t *>(stream_ + offset + 4);
    return { secs, nsecs };
  }
  return values_[index];
}

template<>
ArrayMessage<ros::Duration> *ArrayMessage<ros::Duration>::fromStream( ssize_t length, const uint8_t *stream,
                                                                      size_t stream_length, size_t &bytes_read )
{
  (void) stream_length; // For unused warning
  bool fixed_length = length >= 0;
  stream += bytes_read;
  if ( !fixed_length )
  {
    length = *reinterpret_cast<const uint32_t *>(stream + bytes_read);
    stream += sizeof( uint32_t );
    bytes_read += sizeof( uint32_t );
  }
  bytes_read += 2 * sizeof( int32_t ) * length;
  if ( bytes_read > stream_length )
    throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
  return new ArrayMessage<ros::Duration>( length, fixed_length, stream );
}

template<>
size_t ArrayMessage<ros::Duration>::_sizeInBytes() const
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
  size_t length = _sizeInBytes();
  size_t count = length;
  if ( !fixed_length_ )
  {
    *reinterpret_cast<uint32_t *>(stream) = length_;
    stream += sizeof( uint32_t );
    count -= sizeof( uint32_t );
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

//! ===================================================
//! ================== CompoundArray ==================
//! ===================================================

CompoundArrayMessage *CompoundArrayMessage::fromStream( ssize_t length, MessageTemplate::ConstPtr msg_template,
                                                        const uint8_t *stream, size_t stream_length,
                                                        size_t &bytes_read )
{
  bool fixed_length = length >= 0;
  if ( !fixed_length )
  {
    length = *reinterpret_cast<const uint32_t *>(stream + bytes_read);
    bytes_read += sizeof( uint32_t );
  }
  auto *result = new CompoundArrayMessage( std::move( msg_template ), length, fixed_length, stream );
  for ( ssize_t i = 0; i < length; ++i )
  {
    result->values_.push_back( CompoundMessage::fromStream( result->msg_template_, stream, stream_length, bytes_read ));
  }
  return result;
}

CompoundArrayMessage::CompoundArrayMessage( MessageTemplate::ConstPtr msg_template, size_t length, bool fixed_length,
                                            const uint8_t *stream )
  : ArrayMessage<Message>( MessageTypes::Compound, length, fixed_length, stream )
    , msg_template_( std::move( msg_template ))
{
}

CompoundArrayMessage::CompoundArrayMessage( MessageTemplate::ConstPtr msg_template, size_t length, bool fixed_length )
  : ArrayMessage<Message>( MessageTypes::Compound, length, fixed_length )
    , msg_template_( std::move( msg_template ))
{
  values_.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    values_.push_back( new CompoundMessage( msg_template_ ));
  }
}

Message *CompoundArrayMessage::clone() const
{
  auto result = new CompoundArrayMessage( msg_template_, length(), isFixedSize(), stream_ );
  result->values_.clear();
  std::transform( values_.begin(), values_.end(), std::back_inserter( result->values_ ),
                  []( Message *m ) { return m->clone(); } );
  return result;
}

Message &CompoundArrayMessage::appendEmpty()
{
  if ( fixed_length_ )
  {
    throw BabelFishException( "Can not add items to a fixed size array!" );
  }
  auto m = new CompoundMessage( msg_template_ );
  values_.push_back( m );
  ++length_;
  return *m;
}
}
