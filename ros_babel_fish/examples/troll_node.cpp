// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/messages/array_message.h>
#include <ros_babel_fish/messages/compound_message.h>
#include <ros_babel_fish/messages/value_message.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros/ros.h>

/*!
 * The following example demonstrates how this library can be used to receive messages on one topic, modify elements
 * of the message and republish them on a different topic.
 * In this particular example, the incoming message is searched for string fields and their value is replaced with the
 * lyrics of Rick Astley's Never Gonna Give You Up.
 */

using namespace ros_babel_fish;

void updateMessage( Message &message );

int main( int argc, char **argv )
{
  if ( argc != 3 )
  {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: troll_node [INPUT TOPIC] [OUTPUT_TOPIC]" << std::endl;
    return 1;
  }

  ros::init( argc, argv, "ros_babel_fish_publish_string" );
  ros::NodeHandle nh;
  std::string in_topic = argv[1];
  std::string out_topic = argv[2];
  ros::Publisher pub;
  bool first_message = true;
  BabelFish fish;

  ros::Subscriber sub = nh.subscribe<BabelFishMessage>( in_topic, 1, [ & ]( const BabelFishMessage::ConstPtr &msg )
  {
    if ( first_message )
    {
      pub = fish.advertise( nh, msg->dataType(), out_topic, 10 );
      first_message = false;
    }
    TranslatedMessage::Ptr translated = fish.translateMessage( msg );
    updateMessage( *translated->translated_message );
    BabelFishMessage::Ptr result = fish.translateMessage( translated->translated_message );
    pub.publish( result );
  } );

  ros::spin();
}

static constexpr int song_text_length = 56;
//! The lyrics of Rick Astley's masterpiece "Never Gonna Give You Up" (written and produced by Stock Aitken Waterman)
std::string song_text[song_text_length] = {
  "We're no strangers to love",
  "You know the rules and so do I",
  "A full commitment's what I'm thinking of",
  "You wouldn't get this from any other guy",
  "I just wanna tell you how I'm feeling",
  "Gotta make you understand",
  "Never gonna give you up",
  "Never gonna let you down",
  "Never gonna run around and desert you",
  "Never gonna make you cry",
  "Never gonna say goodbye",
  "Never gonna tell a lie and hurt you",
  "We've known each other for so long",
  "Your heart's been aching but you're too shy to say it",
  "Inside we both know what's been going on",
  "We know the game and we're gonna play it",
  "And if you ask me how I'm feeling",
  "Don't tell me you're too blind to see",
  "Never gonna give you up",
  "Never gonna let you down",
  "Never gonna run around and desert you",
  "Never gonna make you cry",
  "Never gonna say goodbye",
  "Never gonna tell a lie and hurt you",
  "Never gonna give you up",
  "Never gonna let you down",
  "Never gonna run around and desert you",
  "Never gonna make you cry",
  "Never gonna say goodbye",
  "Never gonna tell a lie and hurt you",
  "Never gonna give, never gonna give",
  "(Give you up)",
  "(Ooh) Never gonna give, never gonna give",
  "(Give you up)",
  "We've known each other for so long",
  "Your heart's been aching but you're too shy to say it",
  "Inside we both know what's been going on",
  "We know the game and we're gonna play it",
  "I just wanna tell you how I'm feeling",
  "Gotta make you understand",
  "Never gonna give you up",
  "Never gonna let you down",
  "Never gonna run around and desert you",
  "Never gonna make you cry",
  "Never gonna say goodbye",
  "Never gonna tell a lie and hurt you",
  "Never gonna give you up",
  "Never gonna let you down",
  "Never gonna run around and desert you",
  "Never gonna make you cry",
  "Never gonna say goodbye",
  "Never gonna tell a lie and hurt you",
  "Never gonna give you up",
  "Never gonna let you down",
  "Never gonna run around and desert you",
  "Never gonna make you cry",
};

int song_text_index = 0;

void updateMessage( Message &message )
{
  if ( message.type() == MessageTypes::String )
  {
    message = song_text[song_text_index];
    ++song_text_index;
    if ( song_text_index >= song_text_length ) song_text_index = 0;
  }
  else if ( message.type() == MessageTypes::Array )
  {
    if ( message.as<ArrayMessageBase>().elementType() == MessageTypes::Compound )
    {
      auto &array = message.as<ArrayMessage<Message>>();
      for ( size_t i = 0; i < array.length(); ++i )
      {
        updateMessage( array[i] );
      }
    }
    else if ( message.as<ArrayMessageBase>().elementType() == MessageTypes::String )
    {
      auto &array = message.as<ArrayMessage<std::string>>();
      for ( size_t i = 0; i < array.length(); ++i )
      {
        array.assign( i, song_text[song_text_index] );
        ++song_text_index;
        if ( song_text_index >= song_text_length ) song_text_index = 0;
      }
    }
  }
  else if ( message.type() == MessageTypes::Compound )
  {
    auto &compound = message.as<CompoundMessage>();
    for ( Message *child : compound.values())
    {
      updateMessage( *child );
    }
  }
}
