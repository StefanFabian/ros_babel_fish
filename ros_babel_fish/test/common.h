//
// Created by Stefan Fabian on 17.09.19.
//

#ifndef ROS_BABEL_FISH_TEST_COMMON_H
#define ROS_BABEL_FISH_TEST_COMMON_H

#include <ros_babel_fish_test_msgs/TestArray.h>

#include <random>
#include <vector>

template<typename T>
void fillArray( std::vector<T> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>, std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( distribution( generator ));
  }
}

template<>
void fillArray( std::vector<bool> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<uint8_t> distribution( 0, 1 );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( distribution( generator ) == 1 );
  }
}

template<typename T, size_t L>
void fillArray( boost::array<T, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>, std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
  for ( size_t i = 0; i < L; ++i )
  {
    msg.at( i ) = distribution( generator );
  }
}

template<>
void fillArray<ros::Time>( std::vector<ros::Time> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( 0, 1E9 );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.emplace_back( distribution( generator ));
  }
}

template<size_t L>
void fillArray( boost::array<ros::Time, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( 0, 1E9 );
  for ( size_t i = 0; i < L; ++i )
  {
    msg.at( i ) = ros::Time( distribution( generator ));
  }
}

template<>
void fillArray<ros::Duration>( std::vector<ros::Duration> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( -1E9, 1E9 );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.emplace_back( distribution( generator ));
  }
}

template<size_t L>
void fillArray( boost::array<ros::Duration, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( -1E9, 1E9 );
  for ( size_t i = 0; i < L; ++i )
  {
    msg.at( i ) = ros::Duration( distribution( generator ));
  }
}

std::string randomString( unsigned seed, int length = -1 )
{
  std::default_random_engine generator( seed );
  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";
  std::uniform_int_distribution<size_t> distribution( 0, sizeof( alphanum ) - 2 );
  if ( length == -1 )
  {
    std::uniform_int_distribution<int> length_distribution( 1, 1000 );
    length = length_distribution( generator );
  }
  char result[length + 1];
  for ( int i = 0; i < length; ++i )
  {
    result[i] = alphanum[distribution( generator )];
  }
  result[length] = 0;
  return std::string( result );
}

template<>
void fillArray<std::string>( std::vector<std::string> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min());
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    msg.push_back( randomString( distribution( generator ), i == 0 ? 1 : -1 ));
  }
}

template<size_t L>
void fillArray( boost::array<std::string, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min());
  for ( size_t i = 0; i < msg.length(); ++i )
  {
    msg.at( i ) = randomString( distribution( generator ), i == 0 ? 1 : -1 );
  }
}

template<>
void fillArray<ros_babel_fish_test_msgs::TestSubArray>( std::vector<ros_babel_fish_test_msgs::TestSubArray> &msg,
                                                        unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min());
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i )
  {
    ros_babel_fish_test_msgs::TestSubArray message;
    fillArray( message.ints, seed++ );
    fillArray( message.strings, seed++ );
    fillArray( message.times, seed++ );
    msg.push_back( message );
  }
}

template<size_t L>
void fillArray( boost::array<ros_babel_fish_test_msgs::TestSubArray, L> &msg,
                unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min());
  for ( size_t i = 0; i < L; ++i )
  {
    fillArray( msg[i].ints, seed++ );
    fillArray( msg[i].strings, seed++ );
    fillArray( msg[i].times, seed++ );
  }
}

#endif // ROS_BABEL_FISH_TEST_COMMON_H
