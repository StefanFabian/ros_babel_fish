// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

int main( int argc, char **argv )
{
  if ( argc != 2 )
  {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: service_info [MESSAGE TYPE]" << std::endl;
    std::cout << "Example: service_info std_srvs/Trigger" << std::endl;
    return 1;
  }

  BabelFish babel_fish;
  ServiceDescription::ConstPtr message_description = babel_fish.descriptionProvider()->getServiceDescription( argv[1] );
  if ( message_description == nullptr )
  {
    std::cerr << "No service definition for '" << argv[1] << "' found!" << std::endl;
    return 1;
  }
  std::cout << "Data Type:" << std::endl;
  std::cout << message_description->datatype << std::endl << std::endl;
  std::cout << "MD5:" << std::endl;
  std::cout << message_description->md5 << std::endl << std::endl;
  std::cout << "Service specification:" << std::endl;
  std::cout << "======================" << std::endl;
  std::cout << message_description->specification;
  std::cout << "======================" << std::endl;
  std::cout << "Request:" << std::endl;
  std::cout << "  Data Type:" << std::endl;
  std::cout << "  " << message_description->request->datatype << std::endl;
  std::cout << "  MD5:" << std::endl;
  std::cout << "  " << message_description->request->md5 << std::endl;
  std::cout << "  Message Definition:" << std::endl;
  std::cout << "======================" << std::endl;
  std::cout << "  " << message_description->request->message_definition;
  std::cout << "======================" << std::endl;
  std::cout << "Response:" << std::endl;
  std::cout << "  Data Type:" << std::endl;
  std::cout << "  " << message_description->response->datatype << std::endl;
  std::cout << "  MD5:" << std::endl;
  std::cout << "  " << message_description->response->md5 << std::endl;
  std::cout << "  Message Definition:" << std::endl;
  std::cout << "======================" << std::endl;
  std::cout << "  " << message_description->response->message_definition;
  std::cout << "======================" << std::endl;
  std::cout << std::endl;
}
