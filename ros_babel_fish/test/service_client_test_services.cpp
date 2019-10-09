//
// Created by Stefan Fabian on 04.09.19.
//

#include <ros_babel_fish/babel_fish.h>

#include <roscpp_tutorials/TwoInts.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

bool twoIntServiceCallback( roscpp_tutorials::TwoInts::Request &req, roscpp_tutorials::TwoInts::Response &resp )
{
  resp.sum = req.a + req.b + 42;
  return true;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "service_client_test_services" );
  ros::NodeHandle nh;
  ros::ServiceServer server1 = nh.advertiseService( "/test_service_client/two_ints",
                                                    twoIntServiceCallback );

  BabelFish fish;

  ros::ServiceServer server2 = fish.advertiseService(
    nh, "rosapi/Subscribers", "/test_service_server/subscribers",
    []( Message &req, Message &resp ) -> bool
    {
      resp["subscribers"].as<ArrayMessage<std::string>>().push_back( req["topic"].value<std::string>());
      resp["subscribers"].as<ArrayMessage<std::string>>().push_back( "The answer to everything is:" );

      return true;
    } );

  ros::spin();
  return 0;
}