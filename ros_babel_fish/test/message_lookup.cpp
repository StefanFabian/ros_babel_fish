//
// Created by Stefan Fabian on 15.09.19.
//

#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
#include <ros_babel_fish/generation/providers/message_only_description_provider.h>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <ros_babel_fish_test_msgs/TestMessage.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/ImageMarker.h>
#include <visualization_msgs/InteractiveMarker.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

template<typename MsgType, typename ProviderType>
::testing::AssertionResult compareDescription()
{
  namespace mt = ros::message_traits;
  {
    ProviderType provider;
    MessageDescription::ConstPtr desc = provider.getMessageDescription( mt::DataType<MsgType>::value());
    if ( desc == nullptr )
      return ::testing::AssertionFailure() << "Message description lookup" << std::endl
                                           << "Failed to get message description for: "
                                           << mt::DataType<MsgType>::value();
    if ( mt::MD5Sum<MsgType>::value() != desc->md5 )
      return ::testing::AssertionFailure() << "Message description lookup" << std::endl
                                           << "MD5 Sum differed!" << std::endl
                                           << "BF  MD5: " << desc->md5 << std::endl
                                           << "MSG MD5: " << mt::MD5Sum<MsgType>::value();
    if ( mt::DataType<MsgType>::value() != desc->datatype )
      return ::testing::AssertionFailure() << "Message description lookup" << std::endl
                                           << "Datatype differed!" << std::endl
                                           << "BF  Datatype: " << desc->datatype << std::endl
                                           << "MSG Datatype: " << mt::DataType<MsgType>::value();
    std::string val = mt::Definition<MsgType>::value();
    if ( mt::Definition<MsgType>::value() != desc->message_definition )
      return ::testing::AssertionFailure() << "Message description lookup" << std::endl
                                           << "Definition differed!" << std::endl
                                           << "--------------------------- BF  Definition: ---------------------------"
                                           << std::endl << desc->message_definition << std::endl
                                           << "--------------------------- MSG Definition: ---------------------------"
                                           << std::endl << mt::Definition<MsgType>::value();
  }
  {
    ProviderType provider;
    BabelFishMessage msg;
    msg.morph( mt::MD5Sum<MsgType>::value(), mt::DataType<MsgType>::value(), mt::Definition<MsgType>::value());
    MessageDescription::ConstPtr desc = provider.getMessageDescription( msg );
    if ( desc == nullptr )
      return ::testing::AssertionFailure() << "Message description from message" << std::endl
                                           << "Failed to get message description for: "
                                           << mt::DataType<MsgType>::value();
    if ( mt::MD5Sum<MsgType>::value() != desc->md5 )
      return ::testing::AssertionFailure() << "Message description from message" << std::endl
                                           << "MD5 Sum differed!" << std::endl
                                           << "BF  MD5: " << desc->md5 << std::endl
                                           << "MSG MD5: " << mt::MD5Sum<MsgType>::value();
    if ( mt::DataType<MsgType>::value() != desc->datatype )
      return ::testing::AssertionFailure() << "Message description from message" << std::endl
                                           << "Datatype differed!" << std::endl
                                           << "BF  Datatype: " << desc->datatype << std::endl
                                           << "MSG Datatype: " << mt::DataType<MsgType>::value();
    std::string val = mt::Definition<MsgType>::value();
    if ( mt::Definition<MsgType>::value() != desc->message_definition )
      return ::testing::AssertionFailure() << "Message description from message" << std::endl
                                           << "Definition differed!" << std::endl
                                           << "--------------------------- BF  Definition: ---------------------------"
                                           << std::endl << desc->message_definition << std::endl
                                           << "--------------------------- MSG Definition: ---------------------------"
                                           << std::endl << mt::Definition<MsgType>::value();
  }
  return ::testing::AssertionSuccess();
}

TEST( MessageLookupTest, integratedDescriptionProvider )
{
  EXPECT_TRUE((compareDescription<visualization_msgs::MarkerArray, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<visualization_msgs::Marker, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<visualization_msgs::ImageMarker, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<visualization_msgs::InteractiveMarker, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<visualization_msgs::InteractiveMarkerControl, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<visualization_msgs::MenuEntry, IntegratedDescriptionProvider>()));

  EXPECT_TRUE((compareDescription<geometry_msgs::AccelWithCovarianceStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::AccelWithCovariance, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::AccelStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Accel, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::InertiaStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Inertia, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Point, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::PolygonStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Polygon, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Pose2D, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::PoseArray, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::PoseWithCovarianceStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::PoseWithCovariance, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::PoseStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Pose, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Quaternion, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::TransformStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Transform, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::TwistWithCovarianceStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::TwistWithCovariance, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::TwistStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Twist, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Vector3Stamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Vector3, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::WrenchStamped, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<geometry_msgs::Wrench, IntegratedDescriptionProvider>()));

  EXPECT_TRUE((compareDescription<std_msgs::Bool, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::Byte, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::ByteMultiArray, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::Duration, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::Empty, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::Float32, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::String, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_msgs::Time, IntegratedDescriptionProvider>()));

  EXPECT_TRUE((compareDescription<ros_babel_fish_test_msgs::TestMessage, IntegratedDescriptionProvider>()));
}

TEST( MessageLookupTest, messageOnlyDescriptionProvider )
{
  namespace mt = ros::message_traits;
  MessageOnlyDescriptionProvider provider;
  provider.registerMessageByDefinition( mt::datatype<visualization_msgs::MarkerArray>(),
                                        mt::definition<visualization_msgs::MarkerArray>());
  EXPECT_NO_THROW( provider.getMessageDescription( mt::datatype<visualization_msgs::Marker>()));
  EXPECT_THROW( provider.getMessageDescription( mt::datatype<visualization_msgs::InteractiveMarker>()),
                BabelFishException );
}

TEST( MessageLookupTest, constants )
{
  namespace mt = ros::message_traits;
  MessageOnlyDescriptionProvider provider;
  auto description = provider.registerMessageByDefinition( mt::datatype<ros_babel_fish_test_msgs::TestMessage>(),
                                                           mt::definition<ros_babel_fish_test_msgs::TestMessage>());
  std::map<std::string, Message::ConstPtr> constant_map = description->message_template->constants;
  ASSERT_EQ( constant_map.size(), 4U );
  ASSERT_NE( constant_map.find( "FLAG1" ), constant_map.end());
  ASSERT_EQ( constant_map["FLAG1"]->type(), MessageTypes::Bool );
  ASSERT_EQ( constant_map["FLAG1"]->value<bool>(), ros_babel_fish_test_msgs::TestMessage::FLAG1 );
  ASSERT_NE( constant_map.find( "FLAG2" ), constant_map.end());
  ASSERT_EQ( constant_map["FLAG2"]->type(), MessageTypes::Bool );
  ASSERT_EQ( constant_map["FLAG2"]->value<bool>(), ros_babel_fish_test_msgs::TestMessage::FLAG2 );
  ASSERT_NE( constant_map.find( "FLAG3" ), constant_map.end());
  ASSERT_EQ( constant_map["FLAG3"]->type(), MessageTypes::Bool );
  ASSERT_EQ( constant_map["FLAG3"]->value<bool>(), ros_babel_fish_test_msgs::TestMessage::FLAG3 );
  ASSERT_NE( constant_map.find( "FLAG4" ), constant_map.end());
  ASSERT_EQ( constant_map["FLAG4"]->type(), MessageTypes::Bool );
  ASSERT_EQ( constant_map["FLAG4"]->value<bool>(), ros_babel_fish_test_msgs::TestMessage::FLAG4 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_message_lookup" );
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
