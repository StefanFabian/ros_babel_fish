//
// Created by Stefan Fabian on 17.09.19.
//

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_extractor.h>
#include <ros/ros.h>

#include <chrono>

float sum = 0;
ros_babel_fish::BabelFish fish;
ros_babel_fish::MessageExtractor extractor( fish );
ros_babel_fish::SubMessageLocation location;
std::chrono::time_point<std::chrono::high_resolution_clock> last_message;

void callback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  start = std::chrono::high_resolution_clock::now();
  if ( !location.isValid())
  {
    auto description = fish.descriptionProvider()->getMessageDescription( *msg );
    location = extractor.retrieveLocationForPath( description->message_template, "data" );
  }
  sum += extractor.extractValue<float>( msg, location );
  end = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count() << std::endl;

  last_message = end;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_benchmark_float_rbf_extractor" );
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe<ros_babel_fish::BabelFishMessage>( "/co2", 4000, &callback );
  last_message = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> now = last_message;
  while ( sum == 0 || std::chrono::duration_cast<std::chrono::milliseconds>( now - last_message ).count() < 10000 )
  {
    ros::spinOnce();
    now = std::chrono::high_resolution_clock::now();
  }
  std::cout << sum << std::endl;
  return 0;
}
