//
// Created by Stefan Fabian on 04.09.19.
//

#include "message_comparison.h"

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish_test_msgs/TestArray.h>
#include <ros_babel_fish_test_msgs/TestMessage.h>

#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/topic.h>

#include <random>

using namespace ros_babel_fish;

template<typename T>
void fillArray( ArrayMessage<T> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>, std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
  if ( msg.isFixedSize())
  {
    for ( size_t i = 0; i < msg.length(); ++i )
    {
      msg.assign( i, distribution( generator ));
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( distribution( generator ));
  }
}

template<>
void fillArray<bool>( ArrayMessage<bool> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<uint8_t> distribution( 0, 1 );
  if ( msg.isFixedSize())
  {
    for ( size_t i = 0; i < msg.length(); ++i )
    {
      msg.assign( i, distribution( generator ) == 1 );
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( distribution( generator ) == 1 );
  }
}

template<>
void fillArray<ros::Time>( ArrayMessage<ros::Time> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( 0, 1E9 );
  if ( msg.isFixedSize())
  {
    for ( size_t i = 0; i < msg.length(); ++i )
    {
      msg.assign( i, ros::Time( distribution( generator )));
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( ros::Time( distribution( generator )));
  }
}

template<>
void fillArray<ros::Duration>( ArrayMessage<ros::Duration> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( -1E9, 1E9 );
  if ( msg.isFixedSize())
  {
    for ( size_t i = 0; i < msg.length(); ++i )
    {
      msg.assign( i, ros::Duration( distribution( generator )));
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( ros::Duration( distribution( generator )));
  }
}

std::string randomString( unsigned seed, int length = -1 )
{
  std::default_random_engine generator( seed );
  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";
  std::uniform_int_distribution<size_t> distribution( 0, sizeof( alphanum ) - 2 );
  if ( length == -1 )
  {
    std::uniform_int_distribution<int> length_distribution( 1, 1000 );
    length = length_distribution( generator );
  }
  char result[length + 1];
  for ( int i = 0; i < length; ++i )
  {
    result[i] = alphanum[distribution( generator )];
  }
  result[length] = 0;
  return std::string( result );
}

template<>
void fillArray<std::string>( ArrayMessage<std::string> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min());
  if ( msg.isFixedSize())
  {
    for ( size_t i = 0; i < msg.length(); ++i )
    {
      msg.assign( i, randomString( distribution( generator ), i == 0 ? 1 : -1 ));
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( randomString( distribution( generator ), i == 0 ? 1 : -1 ));
  }
}

template<>
void fillArray<Message>( ArrayMessage<Message> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min());
  if ( msg.isFixedSize())
  {
    for ( size_t i = 0; i < msg.length(); ++i )
    {
      fillArray( msg[i]["ints"].as<ArrayMessage<int32_t >>(), seed++ );
      fillArray( msg[i]["strings"].as<ArrayMessage<std::string >>(), seed++ );
      fillArray( msg[i]["times"].as<ArrayMessage<ros::Time>>(), seed++ );
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.as<CompoundArrayMessage>().appendEmpty();
    fillArray( msg[i]["ints"].as<ArrayMessage<int32_t >>(), seed++ );
    fillArray( msg[i]["strings"].as<ArrayMessage<std::string >>(), seed++ );
    fillArray( msg[i]["times"].as<ArrayMessage<ros::Time>>(), seed++ );
  }
}

class MessageEncodingTest : public ::testing::Test
{
protected:
  void publish( const std::string &topic, const Message::Ptr &msg )
  {
    ros::Publisher pub = fish.advertise( nh_, msg->as<CompoundMessage>().datatype(), topic, 1, true );
    BabelFishMessage::ConstPtr translated_msg = fish.translateMessage( msg );
    pub.publish( translated_msg );
    publishers_.push_back( pub );
  }

  void publish( const std::string &topic, const Message &msg )
  {
    ros::Publisher pub = fish.advertise( nh_, msg.as<CompoundMessage>().datatype(), topic, 1, true );
    BabelFishMessage::ConstPtr translated_msg = fish.translateMessage( msg );
    pub.publish( translated_msg );
    publishers_.push_back( pub );
  }

  void SetUp() override
  {
    std_msgs_header = fish.createMessage( "std_msgs/Header" );
    (*std_msgs_header)["frame_id"] = "babel_fish_frame";
    (*std_msgs_header)["stamp"] = ros::Time( 13.37 );

    geometry_msgs_point = fish.createMessage( "geometry_msgs/Point" );
    (*geometry_msgs_point)["x"] = 1.23;
    (*geometry_msgs_point)["y"] = 2.418;
    (*geometry_msgs_point)["z"] = 4218;

    // Normalized quaternion because unnormalized quaternions should crash programs /s (Still sorry about that)
    geometry_msgs_quaternion = fish.createMessage( "geometry_msgs/Quaternion" );
    (*geometry_msgs_quaternion)["w"] = 0.6325;
    (*geometry_msgs_quaternion)["x"] = 0.123;
    (*geometry_msgs_quaternion)["y"] = 0.1434;
    (*geometry_msgs_quaternion)["z"] = 0.1011;

    geometry_msgs_pose = fish.createMessage( "geometry_msgs/Pose" );
    (*geometry_msgs_pose)["position"] = *geometry_msgs_point;
    (*geometry_msgs_pose)["orientation"] = *geometry_msgs_quaternion;

    geometry_msgs_pose_stamped = fish.createMessage( "geometry_msgs/PoseStamped" );
    (*geometry_msgs_pose_stamped)["header"] = *std_msgs_header;
    (*geometry_msgs_pose_stamped)["pose"] = *geometry_msgs_pose;

    test_msg = fish.createMessage( "ros_babel_fish_test_msgs/TestMessage" );
    (*test_msg)["header"]["stamp"] = ros::Time( 42 );
    (*test_msg)["header"]["frame_id"] = "test";
    (*test_msg)["b"] = false;
    (*test_msg)["ui8"] = 255;
    (*test_msg)["ui16"] = 65535;
    (*test_msg)["ui32"] = 4294967295;
    (*test_msg)["ui64"] = std::numeric_limits<uint64_t>::max();
    (*test_msg)["i8"] = -128;
    (*test_msg)["i16"] = -32768;
    (*test_msg)["i32"] = -2147483648;
    (*test_msg)["i64"] = std::numeric_limits<int64_t>::min();
    (*test_msg)["f32"] = 42.0f;
    (*test_msg)["f64"] = 1337.0;
    (*test_msg)["str"] = "test string";
    (*test_msg)["t"] = ros::Time( 1337, 42 );
    (*test_msg)["d"] = ros::Duration( 42, 1337 );
    auto &cam = (*test_msg)["point_arr"].as<CompoundArrayMessage>();
    for ( int i = 0; i < 5; ++i )
    {
      auto &pose = cam.appendEmpty();
      pose["x"] = i * 0.1;
      pose["y"] = 3 + i * 0.4;
      pose["z"] = 15 + i * 3.14;
    }

    test_array_msg = fish.createMessage( "ros_babel_fish_test_msgs/TestArray" );
    unsigned SEED = 42;
    fillArray((*test_array_msg)["bools"].as<ArrayMessage<bool>>(), SEED++ );
    fillArray((*test_array_msg)["uint8s"].as<ArrayMessage<uint8_t>>(), SEED++ );
    fillArray((*test_array_msg)["uint16s"].as<ArrayMessage<uint16_t>>(), SEED++ );
    fillArray((*test_array_msg)["uint32s"].as<ArrayMessage<uint32_t>>(), SEED++ );
    fillArray((*test_array_msg)["uint64s"].as<ArrayMessage<uint64_t>>(), SEED++ );
    fillArray((*test_array_msg)["int8s"].as<ArrayMessage<int8_t>>(), SEED++ );
    fillArray((*test_array_msg)["int16s"].as<ArrayMessage<int16_t>>(), SEED++ );
    fillArray((*test_array_msg)["int32s"].as<ArrayMessage<int32_t>>(), SEED++ );
    fillArray((*test_array_msg)["int64s"].as<ArrayMessage<int64_t>>(), SEED++ );
    fillArray((*test_array_msg)["float32s"].as<ArrayMessage<float>>(), SEED++ );
    fillArray((*test_array_msg)["float64s"].as<ArrayMessage<double>>(), SEED++ );
    fillArray((*test_array_msg)["times"].as<ArrayMessage<ros::Time>>(), SEED++ );
    fillArray((*test_array_msg)["durations"].as<ArrayMessage<ros::Duration>>(), SEED++ );
    fillArray((*test_array_msg)["strings"].as<ArrayMessage<std::string>>(), SEED++ );
    fillArray((*test_array_msg)["subarrays_fixed"].as<ArrayMessage<Message>>(), SEED++ );
    fillArray((*test_array_msg)["subarrays"].as<ArrayMessage<Message>>(), SEED++ );

    // Publish
    publish( "/test_message_encoding/header", std_msgs_header );
    publish( "/test_message_encoding/point", geometry_msgs_point );
    publish( "/test_message_encoding/quaternion", geometry_msgs_quaternion );
    publish( "/test_message_encoding/pose", geometry_msgs_pose );
    publish( "/test_message_encoding/pose_stamped", geometry_msgs_pose_stamped );
    publish( "/test_message_encoding/test_message", test_msg );
    publish( "/test_message_encoding/test_array", test_array_msg );
    publish( "/test_message_encoding/sub_test_array", (*test_array_msg)["subarrays"].as<CompoundArrayMessage>()[0] );
  }

  ros::NodeHandle nh_;
  std::vector<ros::Publisher> publishers_;

  BabelFish fish;

  Message::Ptr std_msgs_header;
  Message::Ptr geometry_msgs_point;
  Message::Ptr geometry_msgs_quaternion;
  Message::Ptr geometry_msgs_pose;
  Message::Ptr geometry_msgs_pose_stamped;
  Message::Ptr test_array_msg;
  Message::Ptr test_msg;
};


TEST_F( MessageEncodingTest, tests )
{
  using namespace ros::topic;
  auto msg_header = waitForMessage<std_msgs::Header>( "/test_message_encoding/header" );
  ASSERT_EQ( ros::message_traits::DataType<std_msgs::Header>::value(),
             std_msgs_header->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( std_msgs_header, msg_header ));

  auto msg_point = waitForMessage<geometry_msgs::Point>( "/test_message_encoding/point" );
  ASSERT_EQ( ros::message_traits::DataType<geometry_msgs::Point>::value(),
             geometry_msgs_point->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_point, msg_point ));

  auto msg_quaternion = waitForMessage<geometry_msgs::Quaternion>( "/test_message_encoding/quaternion" );
  ASSERT_EQ( ros::message_traits::DataType<geometry_msgs::Quaternion>::value(),
             geometry_msgs_quaternion->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_quaternion, msg_quaternion ));

  auto msg_pose = waitForMessage<geometry_msgs::Pose>( "/test_message_encoding/pose" );
  ASSERT_EQ( ros::message_traits::DataType<geometry_msgs::Pose>::value(),
             geometry_msgs_pose->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_pose, msg_pose ));

  auto msg_pose_stamped = waitForMessage<geometry_msgs::PoseStamped>( "/test_message_encoding/pose_stamped" );
  ASSERT_EQ( ros::message_traits::DataType<geometry_msgs::PoseStamped>::value(),
             geometry_msgs_pose_stamped->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_pose_stamped, msg_pose_stamped ));

  auto msg_test_message = waitForMessage<ros_babel_fish_test_msgs::TestMessage>(
    "/test_message_encoding/test_message" );
  ASSERT_EQ( ros::message_traits::DataType<ros_babel_fish_test_msgs::TestMessage>::value(),
             test_msg->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_msg, msg_test_message ));
}

TEST_F( MessageEncodingTest, arrayTests )
{
  using namespace ros::topic;
  auto msg_header = waitForMessage<std_msgs::Header>( "/test_message_encoding/header" );
  ASSERT_EQ( ros::message_traits::DataType<std_msgs::Header>::value(),
             std_msgs_header->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( std_msgs_header, msg_header ));

  auto msg_sub_test_array = waitForMessage<ros_babel_fish_test_msgs::TestSubArray>(
    "/test_message_encoding/sub_test_array" );
  ASSERT_EQ( ros::message_traits::DataType<ros_babel_fish_test_msgs::TestSubArray>::value(),
             (*test_array_msg)["subarrays"].as<CompoundArrayMessage>()[0].as<CompoundMessage>().datatype());
  EXPECT_TRUE(
    MESSAGE_CONTENT_EQUAL((*test_array_msg)["subarrays"].as<CompoundArrayMessage>()[0], msg_sub_test_array ));

  auto msg_test_array = waitForMessage<ros_babel_fish_test_msgs::TestArray>( "/test_message_encoding/test_array" );
  ASSERT_EQ( ros::message_traits::DataType<ros_babel_fish_test_msgs::TestArray>::value(),
             test_array_msg->as<CompoundMessage>().datatype());
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_array_msg, msg_test_array ));
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_message_decoding" );
  return RUN_ALL_TESTS();
}
