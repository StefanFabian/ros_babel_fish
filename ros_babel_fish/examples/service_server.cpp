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
    nh, "hector_std_msgs/PoseService", "/ros_babel_fish/service",
    []( Message::Ptr &req, Message::Ptr & ) -> bool
    {
      auto &msg = req->as<CompoundMessage>();
      std::cout << "Received request: ";
      std::cout << msg["param"]["position"]["x"].as<ValueMessage<double>>().getValue();
      std::cout << std::endl;
      return true;
    } );
  ros::spin();
}
