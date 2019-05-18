// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/message_types/compound_message.h"
#include "ros_babel_fish/babel_fish.h"
#include "ros_babel_fish/message.h"

namespace ros_babel_fish
{

Message::Message( MessageType type, const uint8_t *stream ) : type_( type ), stream_( stream ) { }

Message &Message::operator[]( const std::string & )
{
  throw BabelFishException(
    "Tried to access child message on message object that does not support child access by key." );
}

const Message &Message::operator[]( const std::string & ) const
{
  throw BabelFishException(
    "Tried to access child message on message object that does not support child access by key." );
}

Message::~Message() = default;
}
