// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
#define ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H

#include "ros_babel_fish/generation/description_provider.h"

namespace ros_babel_fish
{

/**
 * @brief C++ Reimplementation of message look ups and message definition / md5 sum generation
 */
class IntegratedDescriptionProvider : public DescriptionProvider
{
public:
  /**
   * @throws BabelFishException If the package paths could not be obtained which would render the description provider unable to serve its purpose
   */
  IntegratedDescriptionProvider();

protected:
  MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type ) override;

  ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string &type ) override;

  std::map<std::string, std::vector<std::string>> msg_paths_;
  std::map<std::string, std::vector<std::string>> srv_paths_;
};
} // ros_babel_fish

#endif // ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
