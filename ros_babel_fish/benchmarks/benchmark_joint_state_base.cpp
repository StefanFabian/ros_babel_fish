//
// Created by Stefan Fabian on 10.09.19.
//

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <chrono>

double sum = 0;
std::chrono::time_point<std::chrono::high_resolution_clock> last_message;

void callback( const sensor_msgs::JointState::ConstPtr &msg )
{
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  start = std::chrono::high_resolution_clock::now();
  for ( size_t i = 0; i < msg->name.size(); ++i )
  {
    if ( msg->name[i] != "arm_roll_joint" ) continue;

    sum += msg->position[i];
    break;
  }
  end = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count() << std::endl;

  last_message = end;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_benchmark_joint_state_base" );
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe<sensor_msgs::JointState>( "/joint_states", 30000, &callback );
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
