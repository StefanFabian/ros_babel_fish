// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_TEMPLATE_H
#define ROS_BABEL_FISH_MESSAGE_TEMPLATE_H

#include "ros_babel_fish/message.h"

#include <map>
#include <vector>

namespace ros_babel_fish
{

struct MessageTemplate
{
  typedef std::shared_ptr<MessageTemplate> Ptr;
  typedef std::shared_ptr<const MessageTemplate> ConstPtr;

  MessageType type{};
  std::map<std::string, Message::ConstPtr> constants{};
  struct
  {
    std::string datatype;
    std::vector<std::string> names;
    std::vector<MessageTemplate::ConstPtr> types;
  } compound{};
  struct
  {
    /*!
     * Length of the array.
     * -1 if dynamic size.
     */
    ssize_t length;
    /*!
     * Type of the array elements.
     */
    MessageType element_type;
    /*!
     * Element template for compound arrays
     */
    MessageTemplate::ConstPtr element_template;
  } array{};
};
} // ros_babel_fish

#endif //ROS_BABEL_FISH_MESSAGE_TEMPLATE_H
