// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/message_types/array_message.h>
#include <ros_babel_fish/message_types/compound_message.h>
#include <ros_babel_fish/message_types/value_message.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros/ros.h>

/*!
 * The following example demonstrates how messages can be published without knowing their type.
 */

using namespace ros_babel_fish;

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_publish_string" );
  ros::NodeHandle nh;

  BabelFish fish;

  // Publish a string message
  ros::Publisher pub_string = fish.advertise( nh, "std_msgs/String", "/string", 1, true );
  {
    Message::Ptr message = fish.createMessage( "std_msgs/String" );
    (*message)["data"].as<ValueMessage<std::string>>().setValue( "Hello World!" );

    BabelFishMessage::Ptr translated_message = fish.translateMessage( message );
    pub_string.publish( translated_message );
  }

  // Publish a Pose
  ros::Publisher pub_pose = fish.advertise( nh, "geometry_msgs/Pose", "/pose", 1, true );
  {
    Message::Ptr message = fish.createMessage( "geometry_msgs/Pose" );
    auto &compound = message->as<CompoundMessage>();
    compound["position"]["x"].as<ValueMessage<double>>().setValue( 1.1 );
    compound["position"]["y"].as<ValueMessage<double>>().setValue( 2.4 );
    compound["position"]["z"].as<ValueMessage<double>>().setValue( 3.6 );

    compound["orientation"]["w"].as<ValueMessage<double>>().setValue( 0.384 );
    compound["orientation"]["x"].as<ValueMessage<double>>().setValue( -0.003 );
    compound["orientation"]["y"].as<ValueMessage<double>>().setValue( -0.876 );
    compound["orientation"]["z"].as<ValueMessage<double>>().setValue( 0.292 );

    BabelFishMessage::Ptr translated_message = fish.translateMessage( message );
    pub_pose.publish( translated_message );
  }

  BabelFish fish2;
  // Publish a Pose with covariance (has fixed array size)
  ros::Publisher pub_posewcv = fish2.advertise( nh, "geometry_msgs/PoseWithCovariance", "/pose_with_covariance", 1,
                                                true );
  {
    Message::Ptr message = fish2.createMessage( "geometry_msgs/PoseWithCovariance" );
    auto &compound = message->as<CompoundMessage>();
    compound["pose"]["position"]["x"].as<ValueMessage<double>>().setValue( 1.1 );
    compound["pose"]["position"]["y"].as<ValueMessage<double>>().setValue( 2.4 );
    compound["pose"]["position"]["z"].as<ValueMessage<double>>().setValue( 3.1 );


    compound["pose"]["orientation"]["w"].as<ValueMessage<double>>().setValue( 0.384 );
    compound["pose"]["orientation"]["x"].as<ValueMessage<double>>().setValue( -0.003 );
    compound["pose"]["orientation"]["y"].as<ValueMessage<double>>().setValue( -0.876 );
    compound["pose"]["orientation"]["z"].as<ValueMessage<double>>().setValue( 0.292 );

    auto &covariance = compound["covariance"].as<ArrayMessage<double>>();
    for ( size_t i = 0; i < covariance.length(); ++i )
    {
      covariance.setItem( i, static_cast<double>(i));
    }

    BabelFishMessage::Ptr translated_message = fish2.translateMessage( message );
    pub_posewcv.publish( translated_message );
  }

  ROS_INFO( "Staying alive for 3 seconds." );
  ros::WallDuration( 3 ).sleep();
}
