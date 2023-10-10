// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_H
#define ROS_BABEL_FISH_BABEL_FISH_H

#include "ros_babel_fish/generation/description_provider.h"
#include "ros_babel_fish/generation/message_template.h"
#include "ros_babel_fish/babel_fish_message.h"
#include "ros_babel_fish/message_description.h"
#include "ros_babel_fish/message_types.h"

#include <ros/publisher.h>
#include <ros/service_server.h>

namespace ros_babel_fish
{

/*!
 * The Message internally points to the buffer of the BabelFishMessage, hence, this ensures that this buffer is not
 * destroyed as long as the message exists or it is detached from the buffer using Message::detachFromStream.
 */
struct TranslatedMessage
{
  typedef std::shared_ptr<TranslatedMessage> Ptr;
  typedef std::shared_ptr<const TranslatedMessage> ConstPtr;

  TranslatedMessage( IBabelFishMessage::ConstPtr input, Message::Ptr translated )
    : input_message( std::move( input )), translated_message( std::move( translated )) { }

  IBabelFishMessage::ConstPtr input_message;
  Message::Ptr translated_message;
};

/*!
 * Allows communication using message types that are not known at compile time.
 */
class BabelFish
{
public:
  /*!
   * Constructs an instance of BabelFish with a new instance of the default description provider.
   * If you have to use multiple BabelFish instances, it is recommended to shae the description provider to
   * prevent multiple look ups of the same message.
   */
  BabelFish();

  /*!
   * Constructs an instance of BabelFish with the given description provider.
   * @param description_provider The description provider to be used.
   * @throws BabelFishException If the passed description_provider is a nullptr.
   */
  explicit BabelFish( DescriptionProvider::Ptr description_provider );

  ~BabelFish();

  /*!
   * Translates the given BabelFishMessage into a TranslatedMessage containing a reference to the input message and the
   * translated message. The reference to the input message is needed to ensure the data is not destroyed because
   * the translated message may depend on it.
   * @param msg The received BabelFishMessage
   * @return A struct containing the input and the translated message.
   */
  TranslatedMessage::Ptr translateMessage( const IBabelFishMessage::ConstPtr &msg );

  /*!
   * Translates the given BabelFishMessage into a translated message.
   * Since the passed BabelFishMessage is only passed as const reference, BabelFish can not make sure that it is
   * not destroyed during the lifetime of Message (or until Message is detached using Message::detachFromStream).
   * Hence, the user has to ensure the BabelFishMessage is not destroyed or detach the Message before it is destroyed.
   * @param msg The received BabelFishMessage
   * @return The translated message.
   */
  Message::Ptr translateMessage( const IBabelFishMessage &msg );

  /*!
   * Translates a message created by BabelFish into a BabelFishMessage that can be sent using the implementations
   * provided by ROS.
   * @param msg The input message
   * @return The serialized ROS compatible message
   */
  BabelFishMessage::Ptr translateMessage( const Message::ConstPtr &msg );

  /*!
   * @copydoc BabelFish::translateMessage(const Message::ConstPtr&)
   */
  BabelFishMessage::Ptr translateMessage( const Message &msg );

  /*!
   * @copydoc BabelFish::translateMessage(const Message::ConstPtr&)
   * @param latched This bool is passed to the headers
   */
  BabelFishMessage::Ptr translateMessage( const Message &msg, bool latched );

  /*!
   * @copydetails BabelFish::translateMessage(const Message::ConstPtr&)
   * @param msg The input message
   * @param result Container for the serialized ROS compatible message
   * @return True if successful, false otherwise
   */
  bool translateMessage( const Message &msg, BabelFishMessage &result );

  /*!
   * Advertises a publisher on the given topic.
   * @param nh The ros::NodeHandle used to advertise the topic
   * @param type The message type that is advertised, e.g.: "std_msgs/Header"
   * @param topic The topic to publish on
   * @param queue_size The maximum number of outgoing messages to be queued for delivery to subscribers
   * @param latch Whether or not this publisher should latch, i.e., always send out the last message to new subscribers
   * @param connect_cb Function to call whenever a subscriber connects to this topic
   * @param disconnect_cb Function to call whenever a subscriber disconnects from this topic
   * @return A ros::Publisher that can be used to publish BabelFishMessages filled with the given type on the given topic
   */
  ros::Publisher advertise( ros::NodeHandle &nh, const std::string &type, const std::string &topic,
                            uint32_t queue_size, bool latch = false,
                            const ros::SubscriberStatusCallback &connect_cb = ros::SubscriberStatusCallback(),
                            const ros::SubscriberStatusCallback &disconnect_cb = ros::SubscriberStatusCallback());

  /*!
   * Advertises a service on the given topic.
   * @param nh The ros::NodeHandle used to advertise the service
   * @param type The service type that is advertised, e.g.: "rosapi/GetParam"
   * @param service The topic this service is advertised on
   * @param callback The callback to be executed for each service request
   * @return A ros::ServiceServer that can be used to provide a service of the given type on the given topic
   */
  ros::ServiceServer advertiseService( ros::NodeHandle &nh, const std::string &type, const std::string &service,
                                       const std::function<bool( Message &, Message & )> &callback );

  /*!
   * Creates an empty message of the given type.
   * @param type The message type, e.g.: "std_msgs/Header"
   * @return An empty message of the given type
   *
   * @throws BabelFishException If the message description was not found
   */
  Message::Ptr createMessage( const std::string &type );

  /*!
   * Creates a service request message for the given service type.
   * @param type The type of the service, e.g., rosapi/GetParam
   * @return An empty service request message that can be used to call a service of the given type
   *
   * @throws BabelFishException If the service description was not found
   */
  Message::Ptr createServiceRequest( const std::string &type );

  /*!
   * Calls a service on the given topic with the given request
   * @param service
   * @param req
   * @param res
   * @return
   *
   * @throws BabelFishException If the passed req message is not a request
   * @throws BabelFishException If the service description was not found
   */
  bool callService( const std::string &service, const Message::ConstPtr &req, TranslatedMessage::Ptr &res );

  DescriptionProvider::Ptr &descriptionProvider();

private:
  DescriptionProvider::Ptr description_provider_;
};

} // ros_babel_fish

#endif //ROS_BABEL_FISH_BABEL_FISH_H
