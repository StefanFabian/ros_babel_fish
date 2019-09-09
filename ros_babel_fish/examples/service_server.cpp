// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_any_service_server" );
  ros::NodeHandle nh;

  BabelFish fish;

  ros::ServiceServer server = fish.advertiseService(
    nh, "roscpp_tutorials/TwoInts", "/ros_babel_fish/service",
    []( Message &req, Message &res ) -> bool
    {
      auto &msg = req.as<CompoundMessage>();
      std::cout << "Received request: " << std::endl;
      std::cout << "a: " << msg["a"].value<long>() << std::endl;
      std::cout << "b: " << msg["b"].value<long>() << std::endl;
      res["sum"] = 42;
      return true;
    } );
  ros::spin();
}
