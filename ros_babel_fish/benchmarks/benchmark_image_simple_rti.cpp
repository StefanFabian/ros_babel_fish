//
// Created by Stefan Fabian on 10.09.19.
//

#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>

#include <chrono>

long long sum = 0;
RosIntrospection::Parser parser;
std::vector<uint8_t> buffer;
bool registered = false;
std_msgs::Header header;
std::chrono::time_point<std::chrono::high_resolution_clock> last_message;

void callback( const topic_tools::ShapeShifter::ConstPtr &msg )
{
  const std::string &datatype = msg->getDataType();
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  start = std::chrono::high_resolution_clock::now();

  if ( !registered )
  {
    parser.registerMessageDefinition( datatype, RosIntrospection::ROSType( msg->getDataType()),
                                      msg->getMessageDefinition());
    registered = true;
  }
  buffer.resize( msg->size());
  ros::serialization::OStream stream( buffer.data(), buffer.size());
  msg->write( stream );

  absl::Span<uint8_t> buffer_span( buffer );
  header = parser.extractField<std_msgs::Header>( datatype, buffer_span );
  sum += header.seq;

  end = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count() << std::endl;

  last_message = end;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_benchmark_image_rti" );
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe<topic_tools::ShapeShifter>( "/camera360/equirectangular_low_fov", 250,
                                                                        &callback );
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
