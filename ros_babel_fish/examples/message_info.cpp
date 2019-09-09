// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

int main( int argc, char **argv )
{
  if ( argc != 2 )
  {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: message_info [MESSAGE TYPE]" << std::endl;
    std::cout << "Example: message_info std_msgs/Header" << std::endl;
    return 1;
  }

  BabelFish babel_fish;
  MessageDescription::ConstPtr message_description = babel_fish.descriptionProvider()->getMessageDescription( argv[1] );
  if (message_description == nullptr)
  {
    std::cerr << "No message definition for '" << argv[1] << "' found!" << std::endl;
    return 1;
  }
  std::cout << "Data Type:" << std::endl;
  std::cout << message_description->datatype << std::endl << std::endl;
  std::cout << "MD5:" << std::endl;
  std::cout << message_description->md5 << std::endl << std::endl;
  std::cout << "Message Definition:" << std::endl;
  std::cout << message_description->message_definition << std::endl;
  std::cout << std::endl;
}
