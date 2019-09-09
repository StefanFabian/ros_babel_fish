// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/generation/message_template.h>
#include "ros_babel_fish/generation/message_creation.h"

#include "ros_babel_fish/messages/array_message.h"
#include "ros_babel_fish/messages/compound_message.h"
#include "ros_babel_fish/messages/value_message.h"

namespace ros_babel_fish
{

Message *createValueMessageFromDataRaw( MessageType type, const uint8_t *stream, size_t &bytes_read )
{
  switch ( type )
  {
    case MessageTypes::Bool:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Bool>::value>::fromData( stream, bytes_read );
    case MessageTypes::UInt8:
      return ValueMessage<message_type_traits::value_type<MessageTypes::UInt8>::value>::fromData( stream, bytes_read );
    case MessageTypes::UInt16:
      return ValueMessage<message_type_traits::value_type<MessageTypes::UInt16>::value>::fromData( stream, bytes_read );
    case MessageTypes::UInt32:
      return ValueMessage<message_type_traits::value_type<MessageTypes::UInt32>::value>::fromData( stream, bytes_read );
    case MessageTypes::UInt64:
      return ValueMessage<message_type_traits::value_type<MessageTypes::UInt64>::value>::fromData( stream, bytes_read );
    case MessageTypes::Int8:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Int8>::value>::fromData( stream, bytes_read );
    case MessageTypes::Int16:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Int16>::value>::fromData( stream, bytes_read );
    case MessageTypes::Int32:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Int32>::value>::fromData( stream, bytes_read );
    case MessageTypes::Int64:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Int64>::value>::fromData( stream, bytes_read );
    case MessageTypes::Float32:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Float32>::value>::fromData( stream,
                                                                                                    bytes_read );
    case MessageTypes::Float64:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Float64>::value>::fromData( stream,
                                                                                                    bytes_read );
    case MessageTypes::String:
      return ValueMessage<message_type_traits::value_type<MessageTypes::String>::value>::fromData( stream, bytes_read );
    case MessageTypes::Time:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Time>::value>::fromData( stream, bytes_read );
    case MessageTypes::Duration:
      return ValueMessage<message_type_traits::value_type<MessageTypes::Duration>::value>::fromData( stream,
                                                                                                     bytes_read );
    case MessageTypes::Compound:
    case MessageTypes::Array:
      throw BabelFishException( "Array and compound are not value message types!" );
    default:
      throw BabelFishException( "Can not create value message from unknown message type!" );
  }
}

Message::Ptr createValueMessageFromData( MessageType type, const uint8_t *stream, size_t &bytes_read )
{
  return Message::Ptr( createValueMessageFromDataRaw( type, stream, bytes_read ));
}

Message *createMessageFromTemplateRaw( const MessageTemplate &msg_template, const uint8_t *stream, size_t length,
                                       size_t &bytes_read )
{
  if ( msg_template.type == MessageTypes::Compound )
  {
    auto *message = new CompoundMessage( msg_template.compound.datatype, stream );
    for ( size_t i = 0; i < msg_template.compound.names.size(); ++i )
    {
      size_t read = 0;
      message->insert( msg_template.compound.names[i],
                       createMessageFromTemplateRaw( *msg_template.compound.types[i], stream, length, read ));
      bytes_read += read;
      stream += read;
      if ( read > length ) throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
      length -= read;
    }
    return message;
  }
  else if ( msg_template.type == MessageTypes::Array )
  {
    bool fixed_size = msg_template.array.length >= 0;
    size_t array_length = fixed_size ? msg_template.array.length : 0;
    if ( !fixed_size )
    {
      array_length = *reinterpret_cast<const uint32_t *>(stream);
      stream += sizeof( uint32_t );
      bytes_read += sizeof( uint32_t );
      if ( length < sizeof( uint32_t ))
        throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
      length -= sizeof( uint32_t );
    }
    Message *result;
    size_t read = 0;
    switch ( msg_template.array.element_type )
    {
      case MessageTypes::Bool:
        result = new ArrayMessage<bool>( array_length, fixed_size, stream );
        read = array_length * sizeof( uint8_t );
        break;
      case MessageTypes::UInt8:
        result = new ArrayMessage<uint8_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( uint8_t );
        break;
      case MessageTypes::UInt16:
        result = new ArrayMessage<uint16_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( uint16_t );
        break;
      case MessageTypes::UInt32:
        result = new ArrayMessage<uint32_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( uint32_t );
        break;
      case MessageTypes::UInt64:
        result = new ArrayMessage<uint64_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( uint64_t );
        break;
      case MessageTypes::Int8:
        result = new ArrayMessage<int8_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( int8_t );
        break;
      case MessageTypes::Int16:
        result = new ArrayMessage<int16_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( int16_t );
        break;
      case MessageTypes::Int32:
        result = new ArrayMessage<int32_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( int32_t );
        break;
      case MessageTypes::Int64:
        result = new ArrayMessage<int64_t>( array_length, fixed_size, stream );
        read = array_length * sizeof( int64_t );
        break;
      case MessageTypes::Float32:
        result = new ArrayMessage<float>( array_length, fixed_size, stream );
        read = array_length * sizeof( float );
        break;
      case MessageTypes::Float64:
        result = new ArrayMessage<double>( array_length, fixed_size, stream );
        read = array_length * sizeof( double );
        break;
      case MessageTypes::Time:
        result = new ArrayMessage<ros::Time>( array_length, fixed_size, stream );
        read = array_length * 2 * sizeof( uint32_t );
        break;
      case MessageTypes::Duration:
        result = new ArrayMessage<ros::Duration>( array_length, fixed_size, stream );
        read = array_length * 2 * sizeof( int32_t );
        break;
      case MessageTypes::Compound:
      {
        const MessageTemplate::ConstPtr &element_template = msg_template.array.element_template;
        auto *array_message = new CompoundArrayMessage( element_template->compound.datatype, array_length, fixed_size );
        for ( size_t i = 0; i < array_length; ++i )
        {
          array_message->setItem( i,
                                  createMessageFromTemplateRaw( *element_template, stream + read, length - read,
                                                                read ));
          if ( read > length )
            throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
        }
        result = array_message;
        break;
      }
      case MessageTypes::String:
        result = new ArrayMessage<std::string>( array_length, fixed_size, stream );
        read = result->size();
        if ( !fixed_size ) read -= sizeof( uint32_t );
        break;
        // This is actually not supported by the msg format (yet)
//      case MessageTypes::Array:
//      {
//        auto *array_message = new ArrayMessage<Message>( msg_template.array.element_type, array_length, fixed_size );
//        const MessageTemplate::ConstPtr &element_template = msg_template.array.element_template;
//        for ( size_t i = 0; i < array_length; ++i )
//        {
//          array_message->setItem( i,
//                                  createMessageFromTemplateRaw( *element_template, stream + read, length - read,
//                                                                read ));
//          if ( read > length )
//            throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
//        }
//        result = array_message;
//        break;
//      }
      default:
        throw BabelFishException( "Unknown type while decoding message!" );
    }
    if ( read > length ) throw BabelFishException( "Unexpected end of stream while reading message from stream!" );
    bytes_read += read;
    return result;
  }

  size_t read = 0;
  Message *result = createValueMessageFromDataRaw( msg_template.type, stream, read );
  bytes_read += read;
  return result;
}

Message::Ptr createMessageFromTemplate( const MessageTemplate &msg_template, const uint8_t *stream, size_t length,
                                        size_t &bytes_read )
{
  bytes_read = 0;
  if ( stream == nullptr ) return Message::Ptr( createEmptyMessageFromTemplate( msg_template ));
  return Message::Ptr( createMessageFromTemplateRaw( msg_template, stream, length, bytes_read ));
}

Message *createEmptyMessageFromTemplateRaw( const MessageTemplate &msg_template )
{
  if ( msg_template.type == MessageTypes::Compound )
  {
    auto *message = new CompoundMessage( msg_template.compound.datatype );
    for ( size_t i = 0; i < msg_template.compound.names.size(); ++i )
    {
      message->insert( msg_template.compound.names[i],
                       createEmptyMessageFromTemplateRaw( *msg_template.compound.types[i] ));
    }
    return message;
  }
  else if ( msg_template.type == MessageTypes::Array )
  {
    bool fixed_size = msg_template.array.length >= 0;
    size_t array_length = fixed_size ? msg_template.array.length : 0;
    Message *result;
    switch ( msg_template.array.element_type )
    {
      case MessageTypes::Bool:
        result = new ArrayMessage<bool>( array_length, fixed_size );
        break;
      case MessageTypes::UInt8:
        result = new ArrayMessage<uint8_t>( array_length, fixed_size );
        break;
      case MessageTypes::UInt16:
        result = new ArrayMessage<uint16_t>( array_length, fixed_size );
        break;
      case MessageTypes::UInt32:
        result = new ArrayMessage<uint32_t>( array_length, fixed_size );
        break;
      case MessageTypes::UInt64:
        result = new ArrayMessage<uint64_t>( array_length, fixed_size );
        break;
      case MessageTypes::Int8:
        result = new ArrayMessage<int8_t>( array_length, fixed_size );
        break;
      case MessageTypes::Int16:
        result = new ArrayMessage<int16_t>( array_length, fixed_size );
        break;
      case MessageTypes::Int32:
        result = new ArrayMessage<int32_t>( array_length, fixed_size );
        break;
      case MessageTypes::Int64:
        result = new ArrayMessage<int64_t>( array_length, fixed_size );
        break;
      case MessageTypes::Float32:
        result = new ArrayMessage<float>( array_length, fixed_size );
        break;
      case MessageTypes::Float64:
        result = new ArrayMessage<double>( array_length, fixed_size );
        break;
      case MessageTypes::Time:
        result = new ArrayMessage<ros::Time>( array_length, fixed_size );
        break;
      case MessageTypes::Duration:
        result = new ArrayMessage<ros::Duration>( array_length, fixed_size );
        break;
      case MessageTypes::Compound:
      {
        const MessageTemplate::ConstPtr &element_template = msg_template.array.element_template;
        auto *array_message = new CompoundArrayMessage( element_template->compound.datatype, array_length, fixed_size );
        if ( fixed_size )
        {
          for ( size_t i = 0; i < array_length; ++i )
          {
            array_message->setItem( i, createEmptyMessageFromTemplateRaw( *element_template ));
          }
        }
        result = array_message;
        break;
      }
      case MessageTypes::String:
        result = new ArrayMessage<std::string>( array_length, fixed_size );
        break;
        // This is actually not supported by the msg format (yet)
//      case MessageTypes::Array:
//      {
//        auto *array_message = new ArrayMessage<Message>( msg_template.array.element_type, array_length, fixed_size );
//        const MessageTemplate::ConstPtr &element_template = msg_template.array.element_template;
//        for ( size_t i = 0; i < array_length; ++i )
//        {
//          if ( fixed_size )
//          {
//            array_message->setItem( i, createEmptyMessageFromTemplateRaw( *element_template ));
//          }
//          else
//          {
//            array_message->addItem( createEmptyMessageFromTemplateRaw( *element_template ));
//          }
//        }
//        result = array_message;
//        break;
//      }
      default:
        throw BabelFishException( "Unknown type while creating empty message!" );
    }
    return result;
  }

  switch ( msg_template.type )
  {
    case MessageTypes::Bool:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Bool>::value>();
    case MessageTypes::UInt8:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::UInt8>::value>();
    case MessageTypes::UInt16:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::UInt16>::value>();
    case MessageTypes::UInt32:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::UInt32>::value>();
    case MessageTypes::UInt64:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::UInt64>::value>();
    case MessageTypes::Int8:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Int8>::value>();
    case MessageTypes::Int16:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Int16>::value>();
    case MessageTypes::Int32:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Int32>::value>();
    case MessageTypes::Int64:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Int64>::value>();
    case MessageTypes::Float32:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Float32>::value>();
    case MessageTypes::Float64:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Float64>::value>();
    case MessageTypes::String:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::String>::value>();
    case MessageTypes::Time:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Time>::value>();
    case MessageTypes::Duration:
      return new ValueMessage<message_type_traits::value_type<MessageTypes::Duration>::value>();
    default:
      throw BabelFishException( "Can not create value message from unknown message type!" );
  }
}

Message::Ptr createEmptyMessageFromTemplate( const MessageTemplate &msg_template )
{
  return Message::Ptr( createEmptyMessageFromTemplateRaw( msg_template ));
}
}
