// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/messages/array_message.h>
#include <ros_babel_fish/messages/compound_message.h>
#include <ros_babel_fish/messages/value_message.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/babel_fish_message.h>
#include <ros/ros.h>

/*!
 * The following example demonstrates how to subscribe to a topic of any type, retrieve type information such as the
 * topic's datatype the MD% sum and the message definition, and traverse the message's content.
 * The datatype, md5 sum, message definition and the message's content are dumped to the console whenever a new message
 * is received.
 */

void dumpMessageContent( const ros_babel_fish::BabelFishMessage::ConstPtr &msg );

void callback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  std::cout << "Message received!" << std::endl;
  std::cout << "Data Type:" << std::endl;
  std::cout << msg->dataType() << std::endl;
  std::cout << "MD5:" << std::endl;
  std::cout << msg->md5Sum() << std::endl;
  std::cout << "Message Definition:" << std::endl;
  std::cout << msg->definition() << std::endl;
  std::cout << "Message Content:" << std::endl;
  dumpMessageContent( msg );
  std::cout << std::endl;
}


ros_babel_fish::BabelFish *fish;

int main( int argc, char **argv )
{
  if ( argc != 2 )
  {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: any_subscriber [TOPIC]" << std::endl;
    return 1;
  }
  ros::init( argc, argv, "ros_babel_fish_any_subscriber" );
  ros::NodeHandle nh;
  std::string topic = argv[1];

  fish = new ros_babel_fish::BabelFish;
  ros::Subscriber sub = nh.subscribe<ros_babel_fish::BabelFishMessage>( topic, 1, &callback );
  ros::spin();
  delete fish;
}

using namespace ros_babel_fish;

template<typename T>
void printArray( const ArrayMessage<T> &message )
{
  std::cout << "[";
  for ( size_t i = 0; i < message.length(); ++i )
  {
    std::cout << message[i];
    if ( i != message.length() - 1 ) std::cout << ", ";
  }
  std::cout << "]" << std::endl;
}

template<>
void printArray<uint8_t>( const ArrayMessage<uint8_t> &message )
{
  std::cout << "[";
  for ( size_t i = 0; i < message.length(); ++i )
  {
    std::cout << static_cast<unsigned int>(message[i]);
    if ( i != message.length() - 1 ) std::cout << ", ";
  }
  std::cout << "]" << std::endl;
}

void dumpMessageContent( const Message &message, const std::string &prefix = "" )
{
  if ( message.type() == MessageTypes::Compound )
  {
    auto &compound = message.as<CompoundMessage>();
    std::cout << std::endl;
    for ( size_t i = 0; i < compound.keys().size(); ++i )
    {
      std::cout << prefix << compound.keys()[i] << ": ";
      dumpMessageContent( *compound.values()[i], prefix + "  " );
      if ( i != compound.keys().size() - 1 ) std::cout << std::endl;
    }
  }
  else if ( message.type() == MessageTypes::Array )
  {
    auto &base = message.as<ArrayMessageBase>();
    switch ( base.elementType())
    {
      case MessageTypes::None:
        break;
      case MessageTypes::Bool:
        printArray<bool>( base.as<ArrayMessage<bool>>());
        break;
      case MessageTypes::UInt8:
        printArray<uint8_t>( base.as<ArrayMessage<uint8_t>>());
        break;
      case MessageTypes::UInt16:
        printArray<uint16_t>( base.as<ArrayMessage<uint16_t>>());
        break;
      case MessageTypes::UInt32:
        printArray<uint32_t>( base.as<ArrayMessage<uint32_t>>());
        break;
      case MessageTypes::UInt64:
        printArray<uint64_t>( base.as<ArrayMessage<uint64_t>>());
        break;
      case MessageTypes::Int8:
        printArray<int8_t>( base.as<ArrayMessage<int8_t >>());
        break;
      case MessageTypes::Int16:
        printArray<int16_t>( base.as<ArrayMessage<int16_t >>());
        break;
      case MessageTypes::Int32:
        printArray<int32_t>( base.as<ArrayMessage<int32_t >>());
        break;
      case MessageTypes::Int64:
        printArray<int64_t>( base.as<ArrayMessage<int64_t >>());
        break;
      case MessageTypes::Float32:
        printArray<float>( base.as<ArrayMessage<float>>());
        break;
      case MessageTypes::Float64:
        printArray<double>( base.as<ArrayMessage<double>>());
        break;
      case MessageTypes::Time:
        printArray<ros::Time>( base.as<ArrayMessage<ros::Time>>());
        break;
      case MessageTypes::Duration:
        printArray<ros::Duration>( base.as<ArrayMessage<ros::Duration>>());
        break;
      case MessageTypes::String:
        printArray<std::string>( base.as<ArrayMessage<std::string>>());
        break;
      case MessageTypes::Compound:
      case MessageTypes::Array: // Arrays of arrays are actually not supported in the ROS msg format
      {
        std::cout << std::endl;
        auto &array = base.as<ArrayMessage<Message>>();
        for ( size_t i = 0; i < array.length(); ++i )
        {
          std::cout << prefix << "- ";
          dumpMessageContent( array[i], prefix + "  " );
        }
        break;
      }
    }
  }
  else
  {
    switch ( message.type())
    {
      case MessageTypes::Array:
      case MessageTypes::Compound:
      case MessageTypes::None:
        break;
      case MessageTypes::Bool:
        std::cout << (message.as<ValueMessage<bool>>().getValue() ? "true" : "false");
        break;
      case MessageTypes::UInt8:
        std::cout << static_cast<unsigned int>(message.as<ValueMessage<uint8_t>>().getValue());
        break;
      case MessageTypes::UInt16:
        std::cout << message.value<uint16_t>(); // The statement above can be simplified using this convenience method
        break;
      case MessageTypes::UInt32:
        std::cout << message.value<uint32_t>();
        break;
      case MessageTypes::UInt64:
        std::cout << message.value<uint64_t>();
        break;
      case MessageTypes::Int8:
        std::cout << message.value<int8_t>();
        break;
      case MessageTypes::Int16:
        std::cout << message.value<int16_t>();
        break;
      case MessageTypes::Int32:
        std::cout << message.value<int32_t>();
        break;
      case MessageTypes::Int64:
        std::cout << message.value<int64_t>();
        break;
      case MessageTypes::Float32:
        std::cout << message.value<float>();
        break;
      case MessageTypes::Float64:
        std::cout << message.value<double>();
        break;
      case MessageTypes::Time:
        std::cout << message.value<ros::Time>();
        break;
      case MessageTypes::Duration:
        std::cout << message.value<ros::Duration>();
        break;
      case MessageTypes::String:
        std::cout << message.value<std::string>();
        break;
    }
  }
}

void dumpMessageContent( const BabelFishMessage::ConstPtr &msg )
{
  TranslatedMessage::Ptr translated = fish->translateMessage( msg );
  dumpMessageContent( *translated->translated_message );
}
