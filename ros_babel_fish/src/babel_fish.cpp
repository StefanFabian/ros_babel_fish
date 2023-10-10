// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/babel_fish.h"

#include "ros_babel_fish/generation/providers/integrated_description_provider.h"
#include "ros_babel_fish/generation/message_creation.h"
#include "ros_babel_fish/message_types.h"

#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <regex>

namespace ros_babel_fish
{

BabelFish::BabelFish() : BabelFish( std::make_shared<IntegratedDescriptionProvider>()) { }

BabelFish::BabelFish( DescriptionProvider::Ptr description_provider )
  : description_provider_( std::move( description_provider ))
{
  if ( description_provider_ == nullptr )
    throw BabelFishException( "DescriptionProvider passed to BabelFish was nullptr!" );
}

BabelFish::~BabelFish() = default;

TranslatedMessage::Ptr BabelFish::translateMessage( const IBabelFishMessage::ConstPtr &msg )
{
  const MessageDescription::ConstPtr &message_description = description_provider_->getMessageDescription( *msg );
  if ( message_description == nullptr )
  {
    throw BabelFishException(
      "BabelFish failed to get message description for received message of type: " + msg->dataType());
  }
  const MessageTemplate::ConstPtr &msg_template = message_description->message_template;
  const uint8_t *stream = msg->buffer();
  size_t bytes_read = 0;
  if ( stream == nullptr )
  {
    Message::Ptr translated = std::make_shared<CompoundMessage>( msg_template );
    return std::make_shared<TranslatedMessage>( msg, translated );
  }

  Message::Ptr translated( CompoundMessage::fromStream( msg_template, stream, msg->size(), bytes_read ));
  if ( bytes_read != msg->size())
    throw BabelFishException( "Translated message of type '" + msg->dataType() + "' did not consume all message bytes!" );
  return std::make_shared<TranslatedMessage>( msg, translated );
}

Message::Ptr BabelFish::translateMessage( const IBabelFishMessage &msg )
{
  const MessageDescription::ConstPtr &message_description = description_provider_->getMessageDescription( msg );
  if ( message_description == nullptr )
  {
    throw BabelFishException(
      "BabelFish failed to get message description for received message of type: " + msg.dataType());
  }
  const MessageTemplate::ConstPtr &msg_template = message_description->message_template;
  const uint8_t *stream = msg.buffer();
  size_t bytes_read = 0;
  if ( stream == nullptr )
  {
    return std::make_shared<CompoundMessage>( msg_template );
  }

  Message::Ptr translated( CompoundMessage::fromStream( msg_template, stream, msg.size(), bytes_read ));
  if ( bytes_read != msg.size())
    throw BabelFishException( "Translated message of type '" + msg.dataType() + "' did not consume all message bytes!" );
  return translated;
}

BabelFishMessage::Ptr BabelFish::translateMessage( const Message::ConstPtr &msg )
{
  return translateMessage( *msg );
}

BabelFishMessage::Ptr BabelFish::translateMessage( const Message &msg )
{
  return translateMessage( msg, false );
}

BabelFishMessage::Ptr BabelFish::translateMessage( const Message &msg, bool latched )
{
  auto compound_msg = dynamic_cast<const CompoundMessage *>(&msg);
  if ( compound_msg == nullptr )
    throw BabelFishException( "Tried to translate message that is not a compound message!" );

  BabelFishMessage::Ptr result( new BabelFishMessage());
  const MessageDescription::ConstPtr &description = description_provider_->getMessageDescription(
    compound_msg->datatype());
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + compound_msg->datatype());
  }
  result->morph( description->md5, description->datatype, description->message_definition, latched );
  result->allocate( msg._sizeInBytes());
  msg.writeToStream( result->buffer());
  return result;
}

bool BabelFish::translateMessage( const Message &msg, BabelFishMessage &result )
{
  auto compound_msg = dynamic_cast<const CompoundMessage *>(&msg);
  if ( compound_msg == nullptr )
    throw BabelFishException( "Tried to translate message that is not a compound message!" );
  const MessageDescription::ConstPtr &description = description_provider_->getMessageDescription(
    compound_msg->datatype());
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + compound_msg->datatype());
  }
  result.morph( description->md5, description->datatype, description->message_definition, false );
  result.allocate( msg._sizeInBytes());
  msg.writeToStream( result.buffer());
  return true;
}

ros::Publisher BabelFish::advertise( ros::NodeHandle &nh, const std::string &type, const std::string &topic,
                                     uint32_t queue_size, bool latch,
                                     const ros::SubscriberStatusCallback &connect_cb,
                                     const ros::SubscriberStatusCallback &disconnect_cb )
{
  const MessageDescription::ConstPtr &description = description_provider_->getMessageDescription( type );
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  ros::AdvertiseOptions opts( topic, queue_size, description->md5, description->datatype,
                              description->message_definition, connect_cb, disconnect_cb );
  opts.latch = latch;

  return nh.advertise( opts );
}

ros::ServiceServer BabelFish::advertiseService( ros::NodeHandle &nh, const std::string &type,
                                                const std::string &service,
                                                const std::function<bool( Message &, Message & )> &callback )
{
  const ServiceDescription::ConstPtr &description = description_provider_->getServiceDescription( type );
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  ros::AdvertiseServiceOptions opts;
  opts.datatype = description->datatype;
  opts.service = service;
  opts.req_datatype = description->request->datatype;
  opts.res_datatype = description->response->datatype;
  opts.md5sum = description->md5;
  opts.helper = ros::ServiceCallbackHelperPtr(
    new ros::ServiceCallbackHelperT<ros::ServiceSpec<BabelFishMessage, BabelFishMessage>>(
      [ this, &callback ]( BabelFishMessage &req, BabelFishMessage &res ) -> bool
      {
        Message::Ptr translated_req = translateMessage( req );
        Message::Ptr translated_res = translateMessage( res );
        bool result = callback( *translated_req, *translated_res );
        if ( !translateMessage( *translated_res, res ))
        {
          ROS_ERROR_NAMED( "RosBabelFish", "Failed to translate service response!" );
          return false;
        }
        return result;
      },
      [ description ]() -> BabelFishMessage::Ptr
      {
        BabelFishMessage::Ptr message = boost::make_shared<BabelFishMessage>();
        message->morph( description->request, description->md5 );
        return message;
      },
      [ description ]() -> BabelFishMessage::Ptr
      {
        BabelFishMessage::Ptr message = boost::make_shared<BabelFishMessage>();
        message->morph( description->response, description->md5 );
        return message;
      }
    ));
  return nh.advertiseService( opts );
}

Message::Ptr BabelFish::createMessage( const std::string &type )
{
  const MessageDescription::ConstPtr &description = description_provider_->getMessageDescription( type );
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return std::make_shared<CompoundMessage>( description->message_template );
}

Message::Ptr BabelFish::createServiceRequest( const std::string &type )
{
  const ServiceDescription::ConstPtr &description = description_provider_->getServiceDescription( type );
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  return std::make_shared<CompoundMessage>( description->request->message_template );
}

bool BabelFish::callService( const std::string &service, const Message::ConstPtr &req, TranslatedMessage::Ptr &res )
{
  const std::string &datatype = req->as<CompoundMessage>().datatype();
  if ( strcmp( datatype.c_str() + datatype.length() - 7, "Request" ) != 0 )
  {
    throw BabelFishException(  "BabelFish can't call a service with a message that is not a request! "
                              "Message Type: " + datatype );
  }
  const std::string &service_type = datatype.substr( 0, datatype.length() - 7 );
  const ServiceDescription::ConstPtr &description = description_provider_->getServiceDescription( service_type );
  if ( description == nullptr )
  {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + service_type );
  }
  BabelFishMessage::Ptr request = translateMessage( req );
  BabelFishMessage::Ptr response = boost::make_shared<BabelFishMessage>();
  response->morph( description->response );
  bool result = ros::service::call( service, *request, *response );
  res = translateMessage( response );
  return result;
}

DescriptionProvider::Ptr &BabelFish::descriptionProvider()
{
  return description_provider_;
}
}
