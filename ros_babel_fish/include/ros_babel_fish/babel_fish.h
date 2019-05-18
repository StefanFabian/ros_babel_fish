// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_H
#define ROS_BABEL_FISH_BABEL_FISH_H

#include "ros_babel_fish/babel_fish_message.h"
#include "ros_babel_fish/message_description.h"
#include "ros_babel_fish/message_types.h"
#include "ros_babel_fish/message_template.h"

#include <ros/publisher.h>
#include <ros/service_server.h>

namespace ros_babel_fish
{

/*!
 * The Message internally points to the buffer of the BabelFishMessage, hence, this ensures that this buffer is not
 * destroyed as long as the message exists.
 */
struct TranslatedMessage
{
  typedef std::shared_ptr<TranslatedMessage> Ptr;
  typedef std::shared_ptr<const TranslatedMessage> ConstPtr;

  TranslatedMessage( BabelFishMessage::ConstPtr input, Message::Ptr translated )
    : input_message( std::move( input )), translated_message( std::move( translated )) { }

  BabelFishMessage::ConstPtr input_message;
  Message::Ptr translated_message;
};

class BabelFish
{
public:
  BabelFish();

  ~BabelFish();

  const MessageDescription::ConstPtr &getMessageDescription( const std::string &type );

  const ServiceDescription::ConstPtr &getServiceDescription( const std::string &type );

  const MessageTemplate::ConstPtr &getMessageTemplate( const std::string &type );

  TranslatedMessage::Ptr translateMessage( const BabelFishMessage::ConstPtr &msg );

  // TODO Document that this method requires you to make sure msg doesnt get out of scope.
  Message::Ptr translateMessage( const BabelFishMessage &msg );

  BabelFishMessage::Ptr translateMessage( const Message::ConstPtr &msg );

  BabelFishMessage::Ptr translateMessage( const Message &msg );

  ros::Publisher advertise( ros::NodeHandle &nh, const std::string &type, const std::string &topic,
                            uint32_t queue_size_, bool latch = false,
                            const ros::SubscriberStatusCallback &connect_cb = ros::SubscriberStatusCallback());

  ros::ServiceServer advertiseService( ros::NodeHandle &nh, const std::string &type, const std::string &service,
                                       const std::function<bool( Message::Ptr &, Message::Ptr & )> &callback );

  Message::Ptr createMessage( const std::string &type );

  /*!
   * Creates a service request message for the given service type.
   * @param type The type of the service, e.g., rosapi/GetParam
   * @return An empty service request message that can be used to call a service of the given type
   */
  Message::Ptr createServiceRequest( const std::string &type );

  bool callService( const std::string &service, const Message::ConstPtr &req, TranslatedMessage::Ptr &res );

private:
  void initPython();

  void initBuiltInTypes();

  MessageTemplate::Ptr createTemplate( const std::string &type, const std::string &specification );

  ssize_t registerMessage( const std::string &type, const std::string &message_definition,
                           const std::string &md5, const std::string &specification );

  ssize_t registerService( const std::string &type, const std::string &md5, const std::string &specification,
                           const std::string &req_message_definition, const std::string &req_md5,
                           const std::string &req_specification,
                           const std::string &resp_message_definition, const std::string &resp_md5,
                           const std::string &resp_specification );

  ssize_t loadMessage( const std::string &type );

  ssize_t loadService( const std::string &ype );

  void *msg_context_;
  void *msg_search_paths_;
  void *srv_search_paths_;

  // Functions
  void *genmsg_compute_full_text_;
  void *genmsg_compute_md5_;
  void *genmsg_load_depends_;
  void *genmsg_load_msg_by_type_;
  void *genmsg_load_srv_by_type_;

  std::vector<MessageDescription::ConstPtr> message_descriptions_;
  std::vector<ServiceDescription::ConstPtr> service_descriptions_;
  std::set<std::string> builtin_types_;
  std::vector<void *> py_objects_;
};


class BabelFishException : public ros::Exception
{
public:
  explicit BabelFishException( const std::string &msg ) : ros::Exception( msg ) { }
};
} // ros_babel_fish

#endif //ROS_BABEL_FISH_BABEL_FISH_H
