// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_DESCRIPTION_PROVIDER_H
#define ROS_BABEL_FISH_DESCRIPTION_PROVIDER_H

#include "ros_babel_fish/babel_fish_message.h"
#include "ros_babel_fish/message_description.h"

namespace ros_babel_fish
{

class DescriptionProvider
{
public:
  typedef std::shared_ptr<DescriptionProvider> Ptr;

  DescriptionProvider();

  MessageDescription::ConstPtr getMessageDescription( const std::string &type );

  MessageDescription::ConstPtr getMessageDescription( const BabelFishMessage &msg );

  ServiceDescription::ConstPtr getServiceDescription( const std::string &type );

protected:

  virtual MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type ) = 0;

  virtual MessageDescription::ConstPtr getMessageDescriptionImpl( const BabelFishMessage &msg );

  virtual ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string &type ) = 0;

  MessageTemplate::Ptr createTemplate( const std::string &type, const std::string &specification );

  MessageDescription::ConstPtr registerMessage( const std::string &type, const std::string &definition,
                                                const std::string &md5, const std::string &specification );

  ServiceDescription::ConstPtr registerService( const std::string &type, const std::string &md5,
                                                const std::string &specification,
                                                const std::string &req_message_definition,
                                                const std::string &req_md5, const std::string &req_specification,
                                                const std::string &resp_message_definition, const std::string &resp_md5,
                                                const std::string &resp_specification );

private:
  void initBuiltInTypes();

  std::map<std::string, MessageDescription::ConstPtr> message_descriptions_;
  std::map<std::string, ServiceDescription::ConstPtr> service_descriptions_;
  std::set<std::string> builtin_types_;
};
} // ros_babel_fish

#endif // ROS_BABEL_FISH_DESCRIPTION_PROVIDER_H
