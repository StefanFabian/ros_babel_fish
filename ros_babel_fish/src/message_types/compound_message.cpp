// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/message_types/compound_message.h"

namespace ros_babel_fish
{

CompoundMessage::CompoundMessage( std::string datatype, const uint8_t *stream )
  : Message( MessageTypes::Compound, stream ), datatype_( std::move( datatype ))
{
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
  for ( size_t i = 0; i < keys_.size(); ++i )
  {
    if ( keys_[i] == key ) return *values_[i];
  }
  throw std::runtime_error( "Invalid key!" );
}

const Message &CompoundMessage::operator[]( const std::string &key ) const
{
  for ( size_t i = 0; i < keys_.size(); ++i )
  {
    if ( keys_[i] == key ) return *values_[i];
  }
  throw std::runtime_error( "Invalid key!" );
}

size_t CompoundMessage::size() const
{
  size_t result = 0;
  for ( auto &value : values_ )
  {
    result += value->size();
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
}
