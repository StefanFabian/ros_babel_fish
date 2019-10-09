//
// Created by Stefan Fabian on 03.09.19.
//

#include "common.h"
#include "message_comparison.h"

#include <ros_babel_fish/generation/message_creation.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish_test_msgs/TestArray.h>
#include <ros_babel_fish_test_msgs/TestMessage.h>

#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/topic.h>


using namespace ros_babel_fish;

// The messages contain the descriptions, hence no look ups should be performed
class NoLookUpDescriptionProvider : public DescriptionProvider
{
protected:
  MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string & ) override
  {
    return nullptr;
  }

  ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string & ) override
  {
    return nullptr;
  }
};

class MessageDecodingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std_msgs_header.frame_id = "babel_fish_frame";
    std_msgs_header.stamp = ros::Time( 13.37 );

    geometry_msgs_point.x = 1.23;
    geometry_msgs_point.y = 2.418;
    geometry_msgs_point.z = 4218;

    // Normalized quaternion because unnormalized quaternions should crash programs /s (Still sorry about that)
    geometry_msgs_quaternion.w = 0.123;
    geometry_msgs_quaternion.x = 0.4325;
    geometry_msgs_quaternion.y = 0.2011;
    geometry_msgs_quaternion.z = 0.2434;

    geometry_msgs_pose.position = geometry_msgs_point;
    geometry_msgs_pose.orientation = geometry_msgs_quaternion;

    geometry_msgs_pose_stamped.header = std_msgs_header;
    geometry_msgs_pose_stamped.pose = geometry_msgs_pose;

    test_message.header.stamp = ros::Time( 42 );
    test_message.header.frame_id = "test";
    test_message.b = false;
    test_message.ui8 = 255;
    test_message.ui16 = 65535;
    test_message.ui32 = 4294967295;
    test_message.ui64 = std::numeric_limits<uint64_t>::max();
    test_message.i8 = -128;
    test_message.i16 = -32768;
    test_message.i32 = -2147483648;
    test_message.i64 = std::numeric_limits<int64_t>::min();
    test_message.f32 = 42.0;
    test_message.f64 = 1337.0;
    test_message.str = "test string";
    test_message.t = ros::Time( 1337, 42 );
    test_message.d = ros::Duration( 42, 1337 );
    for ( int i = 0; i < 5; ++i )
    {
      geometry_msgs::Point p;
      p.x = i * 0.1;
      p.y = 3 + i * 0.4;
      p.z = 15 + i * 3.14;
      test_message.point_arr.push_back( p );
    }

    unsigned SEED = 42;
    fillArray( test_array.bools, SEED++ );
    fillArray( test_array.uint8s, SEED++ );
    fillArray( test_array.uint16s, SEED++ );
    fillArray( test_array.uint32s, SEED++ );
    fillArray( test_array.uint64s, SEED++ );
    fillArray( test_array.int8s, SEED++ );
    fillArray( test_array.int16s, SEED++ );
    fillArray( test_array.int32s, SEED++ );
    fillArray( test_array.int64s, SEED++ );
    fillArray( test_array.float32s, SEED++ );
    fillArray( test_array.float64s, SEED++ );
    fillArray( test_array.times, SEED++ );
    fillArray( test_array.durations, SEED++ );
    fillArray( test_array.strings, SEED++ );
    fillArray( test_array.subarrays_fixed, SEED++ );
    fillArray( test_array.subarrays, SEED++ );

    header_pub_ = nh_.advertise<std_msgs::Header>( "/test_message_decoding/header", 1, true );
    header_pub_.publish( std_msgs_header );
    point_pub_ = nh_.advertise<geometry_msgs::Point>( "/test_message_decoding/point", 1, true );
    point_pub_.publish( geometry_msgs_point );
    quaternion_pub_ = nh_.advertise<geometry_msgs::Quaternion>( "/test_message_decoding/quaternion", 1, true );
    quaternion_pub_.publish( geometry_msgs_quaternion );
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>( "/test_message_decoding/pose", 1, true );
    pose_pub_.publish( geometry_msgs_pose );
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/test_message_decoding/pose_stamped", 1, true );
    pose_stamped_pub_.publish( geometry_msgs_pose_stamped );
    test_message_pub_ = nh_.advertise<ros_babel_fish_test_msgs::TestMessage>( "/test_message_decoding/test_message", 1,
                                                                              true );
    test_message_pub_.publish( test_message );
    test_sub_array_pub_ = nh_.advertise<ros_babel_fish_test_msgs::TestSubArray>(
      "/test_message_decoding/sub_test_array", 1,
      true );
    test_sub_array_pub_.publish( test_array.subarrays[0] );
    test_array_pub_ = nh_.advertise<ros_babel_fish_test_msgs::TestArray>( "/test_message_decoding/test_array", 1,
                                                                          true );
    test_array_pub_.publish( test_array );
  }

  ros::NodeHandle nh_;
  ros::Publisher header_pub_;
  ros::Publisher point_pub_;
  ros::Publisher quaternion_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_stamped_pub_;
  ros::Publisher test_message_pub_;
  ros::Publisher test_sub_array_pub_;
  ros::Publisher test_array_pub_;

  BabelFish fish = BabelFish( std::make_shared<NoLookUpDescriptionProvider>());

  std_msgs::Header std_msgs_header;
  geometry_msgs::Point geometry_msgs_point;
  geometry_msgs::Quaternion geometry_msgs_quaternion;
  geometry_msgs::Pose geometry_msgs_pose;
  geometry_msgs::PoseStamped geometry_msgs_pose_stamped;
  ros_babel_fish_test_msgs::TestMessage test_message;
  ros_babel_fish_test_msgs::TestArray test_array;
};


TEST_F( MessageDecodingTest, tests )
{
  BabelFishMessage::ConstPtr msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/header" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  EXPECT_THROW( msg->__getServiceDatatype(), BabelFishMessageException );
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( std_msgs_header, msg ));
  TranslatedMessage::Ptr translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( std_msgs_header, translated->translated_message ));

  // Copy constructor
  BabelFishMessage copy = *msg;
  EXPECT_EQ( copy.md5Sum(), msg->md5Sum());
  EXPECT_EQ( copy.size(), msg->size());
  EXPECT_EQ( copy.dataType(), msg->dataType());
  EXPECT_EQ( copy.definition(), msg->definition());
  EXPECT_EQ( copy.isLatched(), msg->isLatched());

  msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/point" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( geometry_msgs_point, msg ));
  translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_point, translated->translated_message ));

  // Copy assignment
  copy = *msg;
  EXPECT_EQ( copy.md5Sum(), msg->md5Sum());
  EXPECT_EQ( copy.size(), msg->size());
  EXPECT_EQ( copy.dataType(), msg->dataType());
  EXPECT_EQ( copy.definition(), msg->definition());
  EXPECT_EQ( copy.isLatched(), msg->isLatched());

  msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/quaternion" );
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( geometry_msgs_quaternion, msg ));
  translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_quaternion, translated->translated_message ));

  msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/pose" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( geometry_msgs_pose, msg ));
  translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_pose, translated->translated_message ));

  msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/pose_stamped" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( geometry_msgs_pose_stamped, msg ));
  translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_pose_stamped, translated->translated_message ));

  msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/test_message" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( test_message, msg ));
  translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_message, translated->translated_message ));

  // Test creation from buffer with invalid length
  MessageDescription::ConstPtr desc = fish.descriptionProvider()->getMessageDescription( *msg );
  size_t bytes_read = 0;
  EXPECT_THROW( createMessageFromTemplate( desc->message_template, msg->buffer(), msg->size() - 1, bytes_read ),
                BabelFishException );
}


TEST_F( MessageDecodingTest, arrayTests )
{
  BabelFishMessage::ConstPtr msg = ros::topic::waitForMessage<BabelFishMessage>(
    "/test_message_decoding/sub_test_array" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( test_array.subarrays[0], msg ));
  TranslatedMessage::Ptr translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_array.subarrays[0], translated->translated_message ));
  EXPECT_FALSE( translated->translated_message->isDetachedFromStream());
  translated->translated_message->detachFromStream();
  EXPECT_TRUE( translated->translated_message->isDetachedFromStream());

  // Test creation from buffer with invalid length
  MessageDescription::ConstPtr desc = fish.descriptionProvider()->getMessageDescription( *msg );
  size_t bytes_read = 0;
  EXPECT_THROW( createMessageFromTemplate( desc->message_template, msg->buffer(), 3, bytes_read ),
                BabelFishException );

  msg = ros::topic::waitForMessage<BabelFishMessage>( "/test_message_decoding/test_array" );
  EXPECT_EQ( msg->dataType(), ros::message_traits::DataType<BabelFishMessage>::value( *msg ));
  ASSERT_TRUE( MESSAGE_TYPE_EQUAL( test_array, msg ));
  translated = fish.translateMessage( msg );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_array, translated->translated_message ));
  translated->translated_message->detachFromStream();
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_array, translated->translated_message ));
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_message_decoding" );
  return RUN_ALL_TESTS();
}
