// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_ONLY_DESCRIPTION_PROVIDER_H
#define ROS_BABEL_FISH_MESSAGE_ONLY_DESCRIPTION_PROVIDER_H

#include "ros_babel_fish/exceptions/babel_fish_exception.h"
#include "ros_babel_fish/generation/description_provider.h"

namespace ros_babel_fish
{

/**
 * @brief Simple and efficient DescriptionProvider that only operates on given messages.
 *
 * This implementation can only be used when receiving messages since it can only extract
 * message definitions from the registered messages.
 * It does not contain to look up unknown messages.
 */
class MessageOnlyDescriptionProvider : public DescriptionProvider
{
public:
  MessageOnlyDescriptionProvider() = default;

  /*!
   * This method registers a message and all its dependencies by its definition.
   *
   * @param type The datatype of the message. Example: std_msgs/Header
   * @param definition The definition of the message. This is the full definition including the the specifications of
   *   all dependencies, i.e., messages this message depends on.
   */
  MessageDescription::ConstPtr registerMessageByDefinition( const std::string &type, const std::string &definition )
  {
    return DescriptionProvider::getMessageDescriptionImpl( type, definition );
  }

  /*!
   * This method registers a message by its specification. This requires all dependencies to be either inbuilt types or
   * registered before.
   *
   * @param type The datatype of the message. Example: std_msgs/Header
   * @param specification The specification of the message. This is simply the specification what this specific message
   *   contains without information about the used messages etc.
   */
  MessageDescription::ConstPtr registerMessageBySpecification( const std::string &type,
                                                const std::string &specification )
  {
    return DescriptionProvider::registerMessage( type, specification );
  }

protected:
  MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type ) override
  {
    throw BabelFishException(
      "This description provider does not support look up for unknown messages and the message definition for " + type +
      " was not registered." );
  }

  ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string &type ) override
  {
    throw BabelFishException(
      "This description provider does not support look up for unknown services and the service definition for " + type +
      " was not registered." );
  }
};
} // ros_babel_fish

#endif //ROS_BABEL_FISH_MESSAGE_ONLY_DESCRIPTION_PROVIDER_H
