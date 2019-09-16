//
// Created by Stefan Fabian on 10.09.19.
//

#include <ros_babel_fish/babel_fish.h>
#include <ros/ros.h>

#include <chrono>

double sum = 0;
ros_babel_fish::BabelFish fish;
std::chrono::time_point<std::chrono::high_resolution_clock> last_message;

void callback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  using namespace ros_babel_fish;
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  start = std::chrono::high_resolution_clock::now();
  TranslatedMessage::Ptr message = fish.translateMessage( msg );
  ArrayMessage<std::string> &arr = (*message->translated_message)["name"].as<ArrayMessage<std::string>>();
  int length = arr.length();
  for ( int i = 0; i < length; ++i )
  {
    if ( arr[i] != "arm_roll_joint" ) continue;
    sum += (*message->translated_message)["position"].as<ArrayMessage<double>>()[i];
    break;
  }
  end = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count() << std::endl;

  last_message = end;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_benchmark_joint_state_rbf" );
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe<ros_babel_fish::BabelFishMessage>( "/joint_states", 30000, &callback );
  last_message = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> now = last_message;
  long time_since_last_message = 0;
  while ( ros::ok() && (sum == 0 || time_since_last_message < 10000))
  {
    ros::spinOnce();
    now = std::chrono::high_resolution_clock::now();
    time_since_last_message = std::chrono::duration_cast<std::chrono::milliseconds>( now - last_message ).count();
  }
  std::cout << sum << std::endl;
  return 0;
}
