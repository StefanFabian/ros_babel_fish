// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/messages/compound_message.h"

#include "ros_babel_fish/exceptions/babel_fish_exception.h"
#include "ros_babel_fish/generation/message_template.h"
#include "ros_babel_fish/messages/array_message.h"
#include "ros_babel_fish/messages/value_message.h"

namespace ros_babel_fish
{


CompoundMessage *CompoundMessage::fromStream( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream,
                                              size_t stream_length, size_t &bytes_read )
{
  using namespace message_type_traits;
  auto *result = new CompoundMessage( msg_template, stream );

  for ( auto &sub_template : msg_template->compound.types )
  {
    switch ( sub_template->type )
    {
      case MessageTypes::Compound:
        result->values_.push_back( CompoundMessage::fromStream( sub_template, stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Bool:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Bool>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::UInt8:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::UInt8>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::UInt16:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::UInt16>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::UInt32:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::UInt32>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::UInt64:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::UInt64>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Int8:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Int8>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Int16:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Int16>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Int32:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Int32>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Int64:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Int64>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Float32:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Float32>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Float64:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Float64>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::String:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::String>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Time:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Time>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Duration:
        result->values_.push_back(
          ValueMessage<value_type<MessageTypes::Duration>::value>::fromStream( stream, stream_length, bytes_read ));
        break;
      case MessageTypes::Array:
      {
        ssize_t length = sub_template->array.length;
        switch ( sub_template->array.element_type )
        {
          case MessageTypes::Bool:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Bool>::value>::fromStream( length, stream, stream_length,
                                                                               bytes_read ));
            break;
          case MessageTypes::UInt8:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::UInt8>::value>::fromStream( length, stream,
                                                                                stream_length, bytes_read ));
            break;
          case MessageTypes::UInt16:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::UInt16>::value>::fromStream( length, stream,
                                                                                 stream_length, bytes_read ));
            break;
          case MessageTypes::UInt32:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::UInt32>::value>::fromStream( length, stream,
                                                                                 stream_length, bytes_read ));
            break;
          case MessageTypes::UInt64:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::UInt64>::value>::fromStream( length, stream,
                                                                                 stream_length, bytes_read ));
            break;
          case MessageTypes::Int8:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Int8>::value>::fromStream( length, stream, stream_length,
                                                                               bytes_read ));
            break;
          case MessageTypes::Int16:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Int16>::value>::fromStream( length, stream,
                                                                                stream_length, bytes_read ));
            break;
          case MessageTypes::Int32:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Int32>::value>::fromStream( length, stream,
                                                                                stream_length, bytes_read ));
            break;
          case MessageTypes::Int64:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Int64>::value>::fromStream( length, stream,
                                                                                stream_length, bytes_read ));
            break;
          case MessageTypes::Float32:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Float32>::value>::fromStream( length, stream,
                                                                                  stream_length, bytes_read ));
            break;
          case MessageTypes::Float64:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Float64>::value>::fromStream( length, stream,
                                                                                  stream_length, bytes_read ));
            break;
          case MessageTypes::String:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::String>::value>::fromStream( length, stream,
                                                                                 stream_length, bytes_read ));
            break;
          case MessageTypes::Time:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Time>::value>::fromStream( length, stream, stream_length,
                                                                               bytes_read ));
            break;
          case MessageTypes::Duration:
            result->values_.push_back(
              ArrayMessage<value_type<MessageTypes::Duration>::value>::fromStream( length, stream,
                                                                                   stream_length, bytes_read ));
            break;
          case MessageTypes::Compound:
            result->values_.push_back(
              CompoundArrayMessage::fromStream( length, sub_template->array.element_template, stream,
                                                stream_length, bytes_read ));
            break;
          case MessageTypes::Array:
          case MessageTypes::None:
            // These don't exist here
            break;
        }
      }
      case MessageTypes::None:
        break;
    }
  }
  return result;
}

CompoundMessage::CompoundMessage( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream )
  : Message( MessageTypes::Compound, stream ), msg_template_( msg_template )
{
  values_.reserve( msg_template->compound.types.size());
}

CompoundMessage::CompoundMessage( const MessageTemplate::ConstPtr &msg_template )
  : Message( MessageTypes::Compound ), msg_template_( msg_template )
{
  using namespace message_type_traits;
  values_.reserve( msg_template->compound.types.size());

  for ( auto &sub_template : msg_template->compound.types )
  {
    switch ( sub_template->type )
    {
      case MessageTypes::Compound:
        values_.push_back( new CompoundMessage( sub_template ));
        break;
      case MessageTypes::Bool:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Bool>::value>());
        break;
      case MessageTypes::UInt8:
        values_.push_back( new ValueMessage<value_type<MessageTypes::UInt8>::value>());
        break;
      case MessageTypes::UInt16:
        values_.push_back( new ValueMessage<value_type<MessageTypes::UInt16>::value>());
        break;
      case MessageTypes::UInt32:
        values_.push_back( new ValueMessage<value_type<MessageTypes::UInt32>::value>());
        break;
      case MessageTypes::UInt64:
        values_.push_back( new ValueMessage<value_type<MessageTypes::UInt64>::value>());
        break;
      case MessageTypes::Int8:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Int8>::value>());
        break;
      case MessageTypes::Int16:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Int16>::value>());
        break;
      case MessageTypes::Int32:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Int32>::value>());
        break;
      case MessageTypes::Int64:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Int64>::value>());
        break;
      case MessageTypes::Float32:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Float32>::value>());
        break;
      case MessageTypes::Float64:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Float64>::value>());
        break;
      case MessageTypes::String:
        values_.push_back( new ValueMessage<value_type<MessageTypes::String>::value>());
        break;
      case MessageTypes::Time:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Time>::value>());
        break;
      case MessageTypes::Duration:
        values_.push_back( new ValueMessage<value_type<MessageTypes::Duration>::value>());
        break;
      case MessageTypes::Array:
      {
        bool fixed_length = sub_template->array.length >= 0;
        size_t length = fixed_length ? sub_template->array.length : 0;
        switch ( sub_template->array.element_type )
        {
          case MessageTypes::Bool:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Bool>::value>( length, fixed_length ));
            break;
          case MessageTypes::UInt8:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::UInt8>::value>( length, fixed_length ));
            break;
          case MessageTypes::UInt16:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::UInt16>::value>( length, fixed_length ));
            break;
          case MessageTypes::UInt32:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::UInt32>::value>( length, fixed_length ));
            break;
          case MessageTypes::UInt64:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::UInt64>::value>( length, fixed_length ));
            break;
          case MessageTypes::Int8:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Int8>::value>( length, fixed_length ));
            break;
          case MessageTypes::Int16:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Int16>::value>( length, fixed_length ));
            break;
          case MessageTypes::Int32:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Int32>::value>( length, fixed_length ));
            break;
          case MessageTypes::Int64:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Int64>::value>( length, fixed_length ));
            break;
          case MessageTypes::Float32:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Float32>::value>( length, fixed_length ));
            break;
          case MessageTypes::Float64:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Float64>::value>( length, fixed_length ));
            break;
          case MessageTypes::String:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::String>::value>( length, fixed_length ));
            break;
          case MessageTypes::Time:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Time>::value>( length, fixed_length ));
            break;
          case MessageTypes::Duration:
            values_.push_back( new ArrayMessage<value_type<MessageTypes::Duration>::value>( length, fixed_length ));
            break;
          case MessageTypes::Compound:
            values_.push_back(
              new CompoundArrayMessage( sub_template->array.element_template, length, fixed_length ));
            break;
          case MessageTypes::Array:
          case MessageTypes::None:
            // These don't exist here
            break;
        }
        break;
      }
      case MessageTypes::None:
        values_.push_back( nullptr );
        break;
    }
  }
}

CompoundMessage::~CompoundMessage()
{
  for ( auto &value : values_ )
  {
    delete value;
  }
  values_.clear();
}

Message &CompoundMessage::operator[]( const std::string &key )
{
  for ( size_t i = 0; i < msg_template_->compound.names.size(); ++i )
  {
    if ( msg_template_->compound.names[i] == key ) return *values_[i];
  }
  throw std::runtime_error( "Invalid key!" );
}

const Message &CompoundMessage::operator[]( const std::string &key ) const
{
  for ( size_t i = 0; i < msg_template_->compound.names.size(); ++i )
  {
    if ( msg_template_->compound.names[i] == key ) return *values_[i];
  }
  throw std::runtime_error( "Invalid key!" );
}

bool CompoundMessage::containsKey( const std::string &key ) const
{
  return std::find( msg_template_->compound.names.begin(), msg_template_->compound.names.end(), key ) !=
         msg_template_->compound.names.end();
}

size_t CompoundMessage::_sizeInBytes() const
{
  size_t result = 0;
  for ( auto &value : values_ )
  {
    result += value->_sizeInBytes();
  }
  return result;
}

bool CompoundMessage::isDetachedFromStream() const
{
  for ( auto &value : values_ )
  {
    if ( !value->isDetachedFromStream()) return false;
  }
  return true;
}

void CompoundMessage::detachFromStream()
{
  for ( auto &value : values_ )
  {
    value->detachFromStream();
  }
}

size_t CompoundMessage::writeToStream( uint8_t *stream ) const
{
  size_t offset = 0;
  for ( auto &value : values_ )
  {
    offset += value->writeToStream( stream + offset );
  }
  return offset;
}

CompoundMessage &CompoundMessage::operator=( const CompoundMessage &other )
{
  stream_ = other.stream_;
  msg_template_ = other.msg_template_;
  for ( auto &value : values_ )
  {
    delete value;
  }
  values_.clear();
  values_.reserve( other.values_.size());
  std::transform( other.values_.begin(), other.values_.end(), std::back_inserter( values_ ),
                  []( Message *m ) { return m->clone(); } );
  return *this;
}

void CompoundMessage::assign( const Message &other )
{
  auto o = dynamic_cast<const CompoundMessage *>(&other);
  if ( o == nullptr ) throw BabelFishException( "Tried to assign incompatible Message type to CompoundMessage!" );
  *this = *o;
}

Message *CompoundMessage::clone() const
{
  // Use overload that does not initialize values
  auto result = new CompoundMessage( msg_template_, nullptr );
  result->values_.reserve( values_.size());
  std::transform( values_.begin(), values_.end(), std::back_inserter( result->values_ ),
                  []( Message *m ) { return m->clone(); } );
  return result;
}
}
