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
  using namespace message_type_traits;
  constexpr size_t limit = std::numeric_limits<size_t>::max();
  switch ( type )
  {
    case MessageTypes::Bool:
      return ValueMessage<typename value_type<MessageTypes::Bool>::value>::fromStream( stream, 1, bytes_read );
    case MessageTypes::UInt8:
      return ValueMessage<typename value_type<MessageTypes::UInt8>::value>::fromStream( stream, 1, bytes_read );
    case MessageTypes::UInt16:
      return ValueMessage<typename value_type<MessageTypes::UInt16>::value>::fromStream( stream, 2, bytes_read );
    case MessageTypes::UInt32:
      return ValueMessage<typename value_type<MessageTypes::UInt32>::value>::fromStream( stream, 4, bytes_read );
    case MessageTypes::UInt64:
      return ValueMessage<typename value_type<MessageTypes::UInt64>::value>::fromStream( stream, 8, bytes_read );
    case MessageTypes::Int8:
      return ValueMessage<typename value_type<MessageTypes::Int8>::value>::fromStream( stream, 1, bytes_read );
    case MessageTypes::Int16:
      return ValueMessage<typename value_type<MessageTypes::Int16>::value>::fromStream( stream, 2, bytes_read );
    case MessageTypes::Int32:
      return ValueMessage<typename value_type<MessageTypes::Int32>::value>::fromStream( stream, 4, bytes_read );
    case MessageTypes::Int64:
      return ValueMessage<typename value_type<MessageTypes::Int64>::value>::fromStream( stream, 8, bytes_read );
    case MessageTypes::Float32:
      return ValueMessage<typename value_type<MessageTypes::Float32>::value>::fromStream( stream, 4, bytes_read );
    case MessageTypes::Float64:
      return ValueMessage<typename value_type<MessageTypes::Float64>::value>::fromStream( stream, 8, bytes_read );
    case MessageTypes::String:
      return ValueMessage<typename value_type<MessageTypes::String>::value>::fromStream( stream, limit, bytes_read );
    case MessageTypes::Time:
      return ValueMessage<typename value_type<MessageTypes::Time>::value>::fromStream( stream, 8, bytes_read );
    case MessageTypes::Duration:
      return ValueMessage<typename value_type<MessageTypes::Duration>::value>::fromStream( stream, 8, bytes_read );
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

Message *createMessageFromTemplateRaw( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream,
                                       size_t length, size_t &bytes_read )
{
  return CompoundMessage::fromStream( msg_template, stream, length, bytes_read );
}

Message::Ptr createMessageFromTemplate( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream,
                                        size_t length, size_t &bytes_read )
{
  bytes_read = 0;
  if ( stream == nullptr ) return Message::Ptr( createEmptyMessageFromTemplate( msg_template ));
  return Message::Ptr( createMessageFromTemplateRaw( msg_template, stream, length, bytes_read ));
}

Message *createEmptyMessageFromTemplateRaw( const MessageTemplate::ConstPtr &msg_template )
{
  return new CompoundMessage( msg_template );
}

Message::Ptr createEmptyMessageFromTemplate( const MessageTemplate::ConstPtr &msg_template )
{
  return Message::Ptr( createEmptyMessageFromTemplateRaw( msg_template ));
}
}
