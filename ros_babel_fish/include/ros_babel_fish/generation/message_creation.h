// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_CREATION_H
#define ROS_BABEL_FISH_MESSAGE_CREATION_H

#include "ros_babel_fish/message.h"
#include "message_template.h"

namespace ros_babel_fish
{

Message::Ptr createValueMessageFromData( MessageType type, const uint8_t *stream, size_t &bytes_read );

Message::Ptr createMessageFromTemplate( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream, size_t length,
                                        size_t &bytes_read );

Message::Ptr createEmptyMessageFromTemplate( const MessageTemplate::ConstPtr &msg_template );
} // ros_babel_fish

#endif //ROS_BABEL_FISH_MESSAGE_CREATION_H
