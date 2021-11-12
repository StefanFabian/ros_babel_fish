// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/message_extractor.h"
#include "ros_babel_fish/exceptions/invalid_message_path_exception.h"
#include "ros_babel_fish/exceptions/invalid_template_exception.h"
#include "ros_babel_fish/generation/message_creation.h"
#include "ros_babel_fish/generation/message_template.h"

namespace ros_babel_fish
{

using OffsetList = std::vector<message_extraction::MessageOffset>;


std::ptrdiff_t message_extraction::MessageOffset::offset( const uint8_t *buffer, std::ptrdiff_t current_offset ) const
{
  switch ( type_ )
  {
    case MessageOffsetTypes::Fixed:
      return offset_;
    case MessageOffsetTypes::String:
      // Length of string in bytes + sizeof length variable
      return *reinterpret_cast<const uint32_t *>(buffer + current_offset) + sizeof( uint32_t );
    case MessageOffsetTypes::Array:
    {
      uint32_t length = *reinterpret_cast<const uint32_t *>(buffer + current_offset);
      if ( array_offsets_.size() == 1 && array_offsets_[0].isFixed())
        return sizeof( uint32_t ) + length * array_offsets_[0].fixedOffset();
      std::ptrdiff_t offset = sizeof( uint32_t );
      size_t array_offsets_size = array_offsets_.size();
      for ( uint32_t i = 0; i < length; ++i )
      {
        for ( size_t k = 0; k < array_offsets_size; ++k )
        {
          offset += array_offsets_[k].offset( buffer, current_offset + offset );
        }
      }
      return offset;
    }
    case MessageOffsetTypes::ArrayElement:
    {
      uint32_t length = *reinterpret_cast<const uint32_t *>(buffer + current_offset);
      if ( length <= index_ ) return -1;
      if ( array_offsets_.size() == 1 && array_offsets_[0].isFixed())
        return sizeof( uint32_t ) + index_ * array_offsets_[0].fixedOffset();
      std::ptrdiff_t offset = sizeof( uint32_t );
      size_t array_offsets_size = array_offsets_.size();
      for ( uint32_t i = 0; i < index_; ++i )
      {
        for ( size_t k = 0; k < array_offsets_size; ++k )
        {
          offset += array_offsets_[k].offset( buffer, current_offset + offset );
        }
      }
      return offset;
    }
  }
  return -1;
}

SubMessageLocation::SubMessageLocation() = default;

SubMessageLocation::SubMessageLocation( std::string root_type, MessageTemplate::ConstPtr msg_template,
                                        std::vector<message_extraction::MessageOffset> offsets )
  : offsets_( std::move( offsets )), msg_template_( std::move( msg_template )), root_type_( std::move( root_type )) { }

std::ptrdiff_t SubMessageLocation::calculateOffset( const IBabelFishMessage &msg ) const
{
  std::ptrdiff_t offset = 0;
  const uint8_t *buffer = msg.buffer();
  uint32_t length = msg.size();
  for ( size_t i = 0; i < offsets_.size(); ++i )
  {
    std::ptrdiff_t sub_offset = offsets_[i].offset( buffer, offset );
    if ( sub_offset < 0 ) return -1;
    offset += sub_offset;
    if ( static_cast<uint32_t>(offset) > length ) return -1;
  }
  return offset;
}

MessageExtractor::MessageExtractor( BabelFish &babel_fish ) : fish_( babel_fish.descriptionProvider()) { }

namespace
{
OffsetList cleanOffsetList( const OffsetList &offset_list )
{
  using namespace message_extraction;
  OffsetList result;
  size_t fixed = 0;
  for ( size_t i = 0; i < offset_list.size(); ++i )
  {
    if ( offset_list[i].isFixed())
    {
      fixed += offset_list[i].offset( nullptr, 0 );
    }
    else
    {
      if ( fixed != 0 ) result.emplace_back( fixed );
      fixed = 0;
      result.push_back( offset_list[i] );
    }
  }
  if ( fixed != 0 ) result.emplace_back( fixed );
  return result;
}

OffsetList getOffset( const MessageTemplate::ConstPtr &msg_template )
{
  using namespace message_extraction;
  switch ( msg_template->type )
  {
    case MessageTypes::Bool:
    case MessageTypes::Int8:
    case MessageTypes::UInt8:
      return { MessageOffset{ 1 }};
    case MessageTypes::Int16:
    case MessageTypes::UInt16:
      return { MessageOffset{ 2 }};
    case MessageTypes::Int32:
    case MessageTypes::UInt32:
    case MessageTypes::Float32:
      return { MessageOffset{ 4 }};
    case MessageTypes::Int64:
    case MessageTypes::UInt64:
    case MessageTypes::Float64:
    case MessageTypes::Time:
    case MessageTypes::Duration:
      return { MessageOffset{ 8 }};
    case MessageTypes::String:
      return { MessageOffset{ MessageOffsetTypes::String }};
    case MessageTypes::Compound:
    {
      OffsetList sub_offsets;
      for ( size_t i = 0; i < msg_template->compound.names.size(); ++i )
      {
        OffsetList sub = getOffset( msg_template->compound.types[i] );
        sub_offsets.insert( sub_offsets.end(), sub.begin(), sub.end());
      }
      return cleanOffsetList( sub_offsets );
    }
    case MessageTypes::Array:
    {
      OffsetList sub_offsets = cleanOffsetList( getOffset( msg_template->array.element_template ));
      if ( sub_offsets.empty()) throw InvalidTemplateException( "Offset list for array elements was empty!" );

      if ( msg_template->array.length != -1 )
      {
        if ( sub_offsets.size() == 1 && sub_offsets[0].isFixed())
          return { MessageOffset{ msg_template->array.length * sub_offsets[0].offset( nullptr, 0 ) }};
        OffsetList result;
        result.reserve( msg_template->array.length * sub_offsets.size());
        for ( ssize_t i = 0; i < msg_template->array.length; ++i )
        {
          result.insert( result.end(), sub_offsets.begin(), sub_offsets.end());
        }
        return cleanOffsetList( result );
      }
      return { MessageOffset{ MessageOffsetTypes::Array, 0, sub_offsets }};
    }
    default:
      throw InvalidTemplateException( "Unknown template type encountered while calculating offset!" );
  }
}
}

SubMessageLocation MessageExtractor::retrieveLocationForPath( const MessageTemplate::ConstPtr &msg_template,
                                                              const std::string &path )
{
  if ( path.empty()) throw InvalidMessagePathException( "Path was empty!" );
  if ( msg_template->type != MessageTypes::Compound )
    throw InvalidTemplateException( "Can only find locations for paths in compounds!" );

  MessageTemplate::ConstPtr sub_template = msg_template;
  std::string::size_type start = path[0] == '.' ? 1 : 0;
  std::string::size_type end;
  OffsetList offsets;
  std::ptrdiff_t fixed_offset = 0;
  while ( true )
  {
    end = path.find( '.', start );
    bool last = end == std::string::npos;
    std::string name = last ? path.substr( start ) : path.substr( start, end - start );
    if ( !last && !(msg_template->type & (MessageTypes::Compound | MessageTypes::Array)))
    {
      throw InvalidMessagePathException(
        "Ended up at leaf at '" + path.substr( 0, end ) + "' but path is '" + path + "'" );
    }
    switch ( sub_template->type )
    {
      case MessageTypes::Bool:
      case MessageTypes::UInt8:
      case MessageTypes::UInt16:
      case MessageTypes::UInt32:
      case MessageTypes::UInt64:
      case MessageTypes::Int8:
      case MessageTypes::Int16:
      case MessageTypes::Int32:
      case MessageTypes::Int64:
      case MessageTypes::Float32:
      case MessageTypes::Float64:
      case MessageTypes::String:
      case MessageTypes::Time:
      case MessageTypes::Duration:
        break;
      case MessageTypes::Compound:
      {
        size_t index;
        for ( index = 0; index < sub_template->compound.names.size(); ++index )
        {
          if ( sub_template->compound.names[index] == name ) break;
          OffsetList sub_list = getOffset( sub_template->compound.types[index] );
          if ( sub_list.size() == 1 && sub_list[0].isFixed())
          {
            fixed_offset += sub_list[0].fixedOffset();
          }
          else
          {
            if ( fixed_offset != 0 ) offsets.emplace_back( fixed_offset );
            fixed_offset = 0;
            offsets.insert( offsets.end(), sub_list.begin(), sub_list.end());
          }
        }
        if ( index == sub_template->compound.names.size())
        {
          throw InvalidMessagePathException(
            "Path '" + path + "' not found, evaluated until '" + path.substr( 0, end ) + "'" );
        }
        sub_template = sub_template->compound.types[index];
        break;
      }
      case MessageTypes::Array:
      {
        ssize_t index;
        try
        {
          index = std::stoul( name );
        } catch ( std::invalid_argument &ex )
        {
          throw InvalidMessagePathException(
            "Invalid index for array '" + path.substr( 0, start - 1 ) + "' in path '" + path + "'!" );
        }
        OffsetList sub_offsets = cleanOffsetList( getOffset( sub_template->array.element_template ));
        if ( sub_offsets.empty()) throw InvalidTemplateException( "Offset list for array elements was empty!" );
        if ( sub_template->array.length != -1 )
        {
          if ( index >= sub_template->array.length )
          {
            throw InvalidMessagePathException( "Path '" + path + "' is invalid because '" +
                                               path.substr( 0, start - 1 ) + "' has a fixed length of " +
                                               std::to_string( sub_template->array.length ) + "!" );
          }
          if ( sub_offsets.size() == 1 && sub_offsets[0].isFixed())
          {
            fixed_offset += index * sub_offsets[0].fixedOffset();
          }
          else
          {
            OffsetList result;
            result.reserve( index * sub_offsets.size());
            for ( ssize_t i = 0; i < index; ++i )
            {
              result.insert( result.end(), sub_offsets.begin(), sub_offsets.end());
            }
            result = cleanOffsetList( result );
            if ( fixed_offset != 0 ) offsets.emplace_back( fixed_offset );
            fixed_offset = 0;
            offsets.insert( offsets.end(), result.begin(), result.end());
          }
        }
        else
        {
          if ( fixed_offset != 0 ) offsets.emplace_back( fixed_offset );
          fixed_offset = 0;
          offsets.emplace_back( message_extraction::MessageOffsetTypes::ArrayElement, 0, sub_offsets, index );
        }
        sub_template = sub_template->array.element_template;
        break;
      }
      default:
        throw InvalidTemplateException( "Unknown template type encountered while calculating offset!" );
    }

    if ( last ) break;
    start = end + 1;
  }
  if ( fixed_offset != 0 ) offsets.emplace_back( fixed_offset );
  return SubMessageLocation{ msg_template->compound.datatype, sub_template, cleanOffsetList( offsets ) };
}

SubMessageLocation MessageExtractor::retrieveLocationForPath( const std::string &base_msg, const std::string &path )
{
  MessageDescription::ConstPtr description = fish_.descriptionProvider()->getMessageDescription( base_msg );
  if ( description == nullptr ) throw BabelFishException( "Failed to lookup msg of type '" + base_msg + "'!" );
  return retrieveLocationForPath( description->message_template, path );
}

SubMessageLocation MessageExtractor::retrieveLocationForPath( const IBabelFishMessage &msg, const std::string &path )
{
  MessageDescription::ConstPtr description = fish_.descriptionProvider()->getMessageDescription( msg );
  if ( description == nullptr ) throw BabelFishException( "Failed to lookup msg of type '" + msg.dataType() + "'!" );
  return retrieveLocationForPath( description->message_template, path );
}

TranslatedMessage::Ptr MessageExtractor::extractMessage( const IBabelFishMessage::ConstPtr &msg,
                                                         const SubMessageLocation &location )
{
  if ( msg->dataType() != location.rootType())
    throw InvalidLocationException( "Location is not valid for this message type!" );
  size_t offset = location.calculateOffset( *msg );
  size_t bytes_read = 0;
  Message::Ptr translated = createMessageFromTemplate( location.messageTemplate(), msg->buffer() + offset,
                                                       msg->size() - offset, bytes_read );
  return std::make_shared<TranslatedMessage>( msg, translated );
}

Message::Ptr MessageExtractor::extractMessage( const IBabelFishMessage &msg, const SubMessageLocation &location )
{
  if ( msg.dataType() != location.rootType())
    throw InvalidLocationException( "Location is not valid for this message type!" );
  size_t offset = location.calculateOffset( msg );
  size_t bytes_read = 0;
  return createMessageFromTemplate( location.messageTemplate(), msg.buffer() + offset, msg.size() - offset,
                                    bytes_read );
}


template<>
std::string MessageExtractor::extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location )
{
  if ( msg.dataType() != location.rootType())
    throw InvalidLocationException( "Location is not valid for this message type!" );
  if ( MessageTypes::String != location.messageTemplate()->type )
    throw BabelFishException( "Tried to extract incompatible type!" );
  std::ptrdiff_t offset = location.calculateOffset( msg );
  if ( offset == -1 ) throw BabelFishException( "Failed to locate submessage!" );
  uint32_t len = *reinterpret_cast<const uint32_t *>(msg.buffer() + offset);
  return std::string( reinterpret_cast<const char *>(msg.buffer() + offset + 4), len );
}

template<>
ros::Time MessageExtractor::extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location )
{
  if ( msg.dataType() != location.rootType())
    throw InvalidLocationException( "Location is not valid for this message type!" );
  if ( MessageTypes::Time != location.messageTemplate()->type )
    throw BabelFishException( "Tried to extract incompatible type!" );
  std::ptrdiff_t offset = location.calculateOffset( msg );
  if ( offset == -1 ) throw BabelFishException( "Failed to locate submessage!" );
  uint32_t secs = *reinterpret_cast<const uint32_t *>(msg.buffer() + offset);
  uint32_t nsecs = *reinterpret_cast<const uint32_t *>(msg.buffer() + offset + 4);
  return { secs, nsecs };
}

template<>
ros::Duration MessageExtractor::extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location )
{
  if ( msg.dataType() != location.rootType())
    throw InvalidLocationException( "Location is not valid for this message type!" );
  if ( MessageTypes::Duration != location.messageTemplate()->type )
    throw BabelFishException( "Tried to extract incompatible type!" );
  std::ptrdiff_t offset = location.calculateOffset( msg );
  if ( offset == -1 ) throw BabelFishException( "Failed to locate submessage!" );
  int32_t secs = *reinterpret_cast<const int32_t *>(msg.buffer() + offset);
  int32_t nsecs = *reinterpret_cast<const int32_t *>(msg.buffer() + offset + 4);
  return { secs, nsecs };
}
} // ros_babel_fish
