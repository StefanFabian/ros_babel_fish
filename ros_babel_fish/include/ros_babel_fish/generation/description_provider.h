// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_DESCRIPTION_PROVIDER_H
#define ROS_BABEL_FISH_DESCRIPTION_PROVIDER_H

#include "ros_babel_fish/babel_fish_message.h"
#include "ros_babel_fish/message_description.h"

#include <unordered_map>

namespace ros_babel_fish
{

/*!
 * Interface for message lookup mechanisms.
 * Responsible for translating the text message definition, md5 sum etc. into a usable format.
 * Caches looked up messages to prevent multiple costly look ups.
 */
class DescriptionProvider
{
protected:
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
  typedef std::shared_ptr<DescriptionProvider> Ptr;

  DescriptionProvider();

  /*!
   * Look up MessageDescription by type.
   * @param type The message type, e.g., std_msgs/Header
   * @return MessageDescription or nullptr if not found.
   */
  MessageDescription::ConstPtr getMessageDescription( const std::string &type );

  /*!
   * Register and create MessageDescription from BabelFishMessage.
   * @param msg The received BabelFishMessage.
   * @return MessageDescription or nullptr if failed.
   *
   * @throws BabelFishException If MessageDescription for message type was already registered but MD5 sums didn't match.
   */
  MessageDescription::ConstPtr getMessageDescription( const BabelFishMessage &msg );

  /*!
   * Look up ServiceDescription by type.
   * @param type The service type, e.g., roscpp_tutorials/TwoInts
   * @return ServiceDescription or nullptr if not found
   */
  ServiceDescription::ConstPtr getServiceDescription( const std::string &type );

  bool isBuiltIn( const std::string &type ) const;

protected:

  /*!
   * Implementation of message look up by type.
   * MessageDescription can be obtained using registerMessage
   * @param type Message type.
   * @return MessageDescription for given type or nullptr if not found
   */
  virtual MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type ) = 0;

  virtual MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type,
                                                                  const std::string &definition );

  /*!
   * Implementation of registration and creation of MessageDescription from BabelFishMessage.
   */
  virtual MessageDescription::ConstPtr getMessageDescriptionImpl( const BabelFishMessage &msg );

  virtual ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string &type ) = 0;

  MessageTemplate::Ptr createTemplate( const MessageSpec &spec );

  MessageDescription::ConstPtr registerMessage( const MessageSpec &spec, const std::string &definition );

  MessageDescription::ConstPtr registerMessage( const std::string &type, const std::string &definition,
                                                const std::string &md5, const std::string &specification );

  MessageDescription::ConstPtr registerMessage( const std::string &type, const std::string &specification );

  ServiceDescription::ConstPtr registerService( const std::string &type, const std::string &md5,
                                                const std::string &specification,
                                                const MessageSpec &req_spec, const std::string &req_definition,
                                                const MessageSpec &resp_spec, const std::string &resp_definition );

  ServiceDescription::ConstPtr registerService( const std::string &type, const std::string &specification,
                                                const std::string &req_specification,
                                                const std::string &resp_specification );

  MessageSpec createSpec( const std::string &type, const std::string &package, const std::string &specification );

  std::vector<std::string> getAllDepends( const MessageSpec &spec );

  void loadDependencies( const MessageSpec &spec );

  std::string computeFullText( const MessageSpec &spec );

  std::string computeMD5Text( const MessageSpec &spec );

private:
  void initBuiltInTypes();


  std::unordered_map<std::string, const MessageSpec> msg_specs_;
  std::unordered_map<std::string, MessageDescription::ConstPtr> message_descriptions_;
  std::unordered_map<std::string, ServiceDescription::ConstPtr> service_descriptions_;
  std::set<std::string> builtin_types_;
};
} // ros_babel_fish

#endif // ROS_BABEL_FISH_DESCRIPTION_PROVIDER_H
