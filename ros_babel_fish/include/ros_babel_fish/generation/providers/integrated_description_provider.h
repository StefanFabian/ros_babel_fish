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
  struct MessageSpec
  {
    struct Constant
    {
      std::string type;
      std::string name;
      std::string val;
    };
    std::string name;
    std::string package;
    std::string text;
    std::vector<Constant> constants;
    std::vector<std::string> types;
    std::vector<std::string> names;
    std::vector<std::string> dependencies;
    std::string md5;
  };
public:
  /**
   * @throws BabelFishException If the package paths could not be obtained which would render the description provider unable to serve its purpose
   */
  IntegratedDescriptionProvider();

protected:
  MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type ) override;

  MessageDescription::ConstPtr getMessageDescriptionImpl( const BabelFishMessage &msg ) override;

  ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string &type ) override;

  MessageSpec createSpec( const std::string &type, const std::string &package, const std::string &specification );

  void loadDependencies( const MessageSpec &spec );

  std::vector<std::string> getAllDepends( const MessageSpec &spec );

  std::string computeFullText( const MessageSpec &spec );

  std::string computeMD5Text( const MessageSpec &spec );

  std::map<std::string, std::string> msg_paths_;
  std::map<std::string, std::string> srv_paths_;

  std::map<std::string, const MessageSpec> msg_specs_;
};
} // ros_babel_fish

#endif // ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
