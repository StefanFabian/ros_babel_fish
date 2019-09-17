// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/messages/compound_message.h"
#include "ros_babel_fish/exceptions/babel_fish_exception.h"

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

bool CompoundMessage::containsKey( const std::string &key ) const
{
  return std::find( keys_.begin(), keys_.end(), key ) != keys_.end();
}

void CompoundMessage::insert( const std::string &key, Message *value )
{
  for ( size_t i = 0; i < keys_.size(); ++i )
  {
    if ( keys_[i] == key )
    {
      delete values_[i];
      values_[i] = value;
      return;
    }
  }
  keys_.push_back( key );
  values_.push_back( value );
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

CompoundMessage &CompoundMessage::operator=( const CompoundMessage &other )
{
  datatype_ = other.datatype_;
  keys_.clear();
  keys_ = other.keys_;
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
  auto result = new CompoundMessage( datatype_ );
  result->keys_ = keys_;
  result->values_.reserve( values_.size());
  std::transform( values_.begin(), values_.end(), std::back_inserter( result->values_ ),
                  []( Message *m ) { return m->clone(); } );
  return result;
}
}
