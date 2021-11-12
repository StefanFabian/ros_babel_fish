//
// Created by Stefan Fabian on 17.09.19.
//

#include "common.h"
#include "message_comparison.h"

#include <ros_babel_fish/exceptions/invalid_message_path_exception.h>
#include <ros_babel_fish/exceptions/invalid_template_exception.h>
#include <ros_babel_fish/message_extractor.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros_babel_fish/generation/message_template.h>

using namespace ros_babel_fish;

TEST( MessageExtractorTest, calculateOffset )
{
  BabelFish fish;
  MessageExtractor extractor( fish );
  EXPECT_THROW( extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", "..subarrays" ),
                InvalidMessagePathException );
  {
    SubMessageLocation location;
    EXPECT_FALSE( location.isValid());
    location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", "bools" );
    EXPECT_TRUE( location.isValid());
    EXPECT_EQ( location.calculateOffset( BabelFishMessage()), 0 );
    ASSERT_EQ( location.messageTemplate()->type, MessageTypes::Array );
    EXPECT_EQ( location.messageTemplate()->array.element_type, MessageTypes::Bool );
  }
  {
    SubMessageLocation location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", ".bools" );
    EXPECT_EQ( location.calculateOffset( BabelFishMessage()), 0 );
    ASSERT_EQ( location.messageTemplate()->type, MessageTypes::Array );
    EXPECT_EQ( location.messageTemplate()->array.element_type, MessageTypes::Bool );
  }
}

TEST( MessageExtractorTest, extractValue )
{
  BabelFish fish;
  MessageExtractor extractor( fish );
  ros_babel_fish_test_msgs::TestArray msg;
  unsigned SEED = 42;
  fillArray( msg.bools, SEED++ );
  fillArray( msg.uint8s, SEED++ );
  fillArray( msg.uint16s, SEED++ );
  fillArray( msg.uint32s, SEED++ );
  fillArray( msg.uint64s, SEED++ );
  fillArray( msg.int8s, SEED++ );
  fillArray( msg.int16s, SEED++ );
  fillArray( msg.int32s, SEED++ );
  fillArray( msg.int64s, SEED++ );
  fillArray( msg.float32s, SEED++ );
  fillArray( msg.float64s, SEED++ );
  fillArray( msg.times, SEED++ );
  fillArray( msg.durations, SEED++ );
  fillArray( msg.strings, SEED++ );
  fillArray( msg.subarrays_fixed, SEED++ );
  fillArray( msg.subarrays, SEED++ );
  EXPECT_THROW(
    extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", ".subarrays_fixed.4.times.42" ),
    InvalidMessagePathException );
  EXPECT_THROW(
    extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", ".subarrays_fixed.4.timers.13" ),
    InvalidMessagePathException );
  EXPECT_THROW(
    extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", ".subarrays_fixed.4.times.abc" ),
    InvalidMessagePathException );
  SubMessageLocation location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray",
                                                                   ".subarrays_fixed.4.times.13" );

  msg.subarrays_fixed[4].times[13] = ros::Time( 13, 37 );
  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  BabelFishMessage bf_msg;
  bf_msg.morph( fish.descriptionProvider()->getMessageDescription( "ros_babel_fish_test_msgs/TestArray" ));
  ros::serialization::deserializeMessage( serialized_msg, bf_msg );
  EXPECT_EQ( extractor.extractValue<ros::Time>( bf_msg, location ), ros::Time( 13, 37 ));
  msg.subarrays_fixed[4].times[13] = ros::Time( 42, 96 );
  size_t int_size = msg.int32s.size();
  msg.int32s.emplace_back( 1 );
  msg.int32s.emplace_back( 2 );
  msg.strings.emplace_back( "Hopefully not the same as the random string :)" );
  serialized_msg = ros::serialization::serializeMessage( msg );
  ros::serialization::deserializeMessage( serialized_msg, bf_msg );
  EXPECT_EQ( extractor.extractValue<ros::Time>( bf_msg, location ), ros::Time( 42, 96 ));
  EXPECT_THROW( extractor.extractValue<ros::Duration>( bf_msg, location ), BabelFishException );
  location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray",
                                                ".int32s." + std::to_string( int_size ));
  EXPECT_EQ( extractor.extractValue<int32_t>( bf_msg, location ), 1 );
  location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray",
                                                ".int32s." + std::to_string( int_size + 1 ));
  EXPECT_EQ( extractor.extractValue<int32_t>( bf_msg, location ), 2 );
  location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray",
                                                ".strings." + std::to_string( msg.strings.size() - 1 ));
  EXPECT_EQ( extractor.extractValue<std::string>( bf_msg, location ), msg.strings[msg.strings.size() - 1] );
  EXPECT_THROW( extractor.extractValue<ros::Time>( bf_msg, location ), BabelFishException );
  location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestArray", ".durations.3" );
  EXPECT_EQ( extractor.extractValue<ros::Duration>( bf_msg, location ), msg.durations[3] );
  EXPECT_THROW( extractor.extractValue<ros::Time>( bf_msg, location ), BabelFishException );
  EXPECT_THROW( extractor.extractValue<std::string>( bf_msg, location ), BabelFishException );
  EXPECT_THROW( extractor.extractValue<float>( bf_msg, location ), BabelFishException );

  MessageDescription::ConstPtr description = fish.descriptionProvider()->getMessageDescription(
    "ros_babel_fish_test_msgs/TestArray" );
  location = extractor.retrieveLocationForPath( description->message_template, ".subarrays.4" );
  {
    Message::Ptr translated = extractor.extractMessage( bf_msg, location );
    EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( msg.subarrays[4], translated ));
    EXPECT_THROW( extractor.retrieveLocationForPath( description->message_template->compound.types[3], ".4" ),
                  InvalidTemplateException );
  }
  BabelFishMessage::Ptr bf_msg_ptr( new BabelFishMessage );
  bf_msg_ptr->morph( fish.descriptionProvider()->getMessageDescription( "ros_babel_fish_test_msgs/TestArray" ));
  ros::serialization::deserializeMessage( serialized_msg, *bf_msg_ptr );
  {
    TranslatedMessage::Ptr translated = extractor.extractMessage( bf_msg_ptr, location );
    EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( msg.subarrays[4], translated->translated_message ));
  }

  location = extractor.retrieveLocationForPath( "ros_babel_fish_test_msgs/TestSubArray", "times.4" );
  EXPECT_TRUE( location.isValid());
  EXPECT_THROW( extractor.extractValue<uint8_t>( bf_msg, location ), InvalidLocationException );
  EXPECT_THROW( extractor.extractValue<ros::Time>( bf_msg, location ), InvalidLocationException );
  EXPECT_THROW( extractor.extractValue<ros::Duration>( bf_msg, location ), InvalidLocationException );
  EXPECT_THROW( extractor.extractValue<std::string>( bf_msg, location ), InvalidLocationException );
  EXPECT_THROW( extractor.extractMessage( bf_msg, location ), InvalidLocationException );
  EXPECT_THROW( extractor.extractMessage( bf_msg_ptr, location ), InvalidLocationException );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_message_lookup" );
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
