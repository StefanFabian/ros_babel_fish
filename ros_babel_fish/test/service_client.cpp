//
// Created by Stefan Fabian on 04.09.19.
//

#include "message_comparison.h"

#include <ros_babel_fish/babel_fish.h>

#include <rosapi/Subscribers.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

TEST( ServiceClientTest, tests )
{
  ros::service::waitForService( "/test_service_client/two_ints" );
  BabelFish fish;
  Message::Ptr req = fish.createServiceRequest( "roscpp_tutorials/TwoInts" );
  (*req)["a"] = 512;
  (*req)["b"] = 314;
  TranslatedMessage::Ptr res;
  ASSERT_TRUE( fish.callService( "/test_service_client/two_ints", req, res ));
  EXPECT_EQ((*res->translated_message)["sum"].value<int64_t>(), 868 ); // Sum is 512 + 314 + 42 = 868
  EXPECT_EQ( res->input_message->__getServiceDatatype(), "roscpp_tutorials/TwoInts" );
}

TEST( ServiceTest, server )
{
  ros::service::waitForService( "/test_service_server/subscribers" );
  rosapi::Subscribers subscribers;
  subscribers.request.topic = "First test";
  EXPECT_TRUE( ros::service::call( "/test_service_server/subscribers", subscribers ));
  ASSERT_EQ( subscribers.response.subscribers.size(), 2UL );
  EXPECT_EQ( subscribers.response.subscribers[0], "First test" );
  EXPECT_EQ( subscribers.response.subscribers[1], "The answer to everything is:" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_service_client" );
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
