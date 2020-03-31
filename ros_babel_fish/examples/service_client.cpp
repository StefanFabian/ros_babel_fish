// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_any_service_client" );
  ros::NodeHandle nh;

  BabelFish fish;
  Message::Ptr req = fish.createServiceRequest( "roscpp_tutorials/TwoInts" );
  req->as<CompoundMessage>()["a"] = 314;
  (*req)["b"] = 1337;
  TranslatedMessage::Ptr res;
  std::cout << "Call result: " << fish.callService( "/ros_babel_fish/service", req, res ) << std::endl;
  std::cout << "Response:" << std::endl;
  std::cout << "  sum: " << res->translated_message->as<CompoundMessage>()["sum"].value<int64_t>() << std::endl;
}
