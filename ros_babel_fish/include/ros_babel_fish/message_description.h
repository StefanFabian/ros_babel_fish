// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_DESCRIPTION_H
#define ROS_BABEL_FISH_MESSAGE_DESCRIPTION_H

#include "ros_babel_fish/generation/message_template.h"

namespace ros_babel_fish
{

struct MessageDescription
{
  typedef std::shared_ptr<MessageDescription> Ptr;
  typedef std::shared_ptr<const MessageDescription> ConstPtr;

  std::string datatype;
  std::string md5;
  std::string message_definition;
  std::string specification;

  MessageTemplate::ConstPtr message_template;
};

struct ServiceDescription
{
  typedef std::shared_ptr<ServiceDescription> Ptr;
  typedef std::shared_ptr<const ServiceDescription> ConstPtr;

  std::string datatype;
  std::string md5;
  std::string specification;

  MessageDescription::ConstPtr request;
  MessageDescription::ConstPtr response;
};
} // ros_babel_fish

#endif //ROS_BABEL_FISH_MESSAGE_DESCRIPTION_H
