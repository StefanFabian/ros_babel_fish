// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/actionlib/client/simple_action_client.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_any_service_client" );
  ros::NodeHandle nh;

  BabelFish fish;
  ros_babel_fish::MessageDescription::ConstPtr goal_description = fish.descriptionProvider()->getMessageDescription(
    "actionlib_tutorials/FibonacciActionGoal" );
  actionlib::SimpleActionClient<BabelFishAction> client( goal_description, "fibonacci" );
  ROS_INFO( "Waiting for server to come up." );
  if ( !client.waitForServer())
  {
    ROS_ERROR( "Could not connect to server!" );
    return 1;
  }
  // Create and translate goal
  Message::Ptr goal = fish.createMessage( "actionlib_tutorials/FibonacciGoal" );
  (*goal)["order"] = 7;
  BabelFishMessage::ConstPtr msg = fish.translateMessage( goal );

  ROS_INFO( "Sending goal." );
  actionlib::SimpleClientGoalState goal_state = client.sendGoalAndWait( *msg );
  std::cout << "Goal state: " << goal_state.getText() << std::endl;

  // Obtain result
  TranslatedMessage::Ptr result = fish.translateMessage( client.getResult());
  auto &sequence = (*result->translated_message)["sequence"].as<ArrayMessage<int32_t> >();

  // Print result sequence
  std::cout << "Result is an array of length:" << sequence.length() << std::endl;
  for ( size_t i = 0; i < sequence.length(); ++i )
  {
    std::cout << sequence[i];
    if ( i != sequence.length() - 1 ) std::cout << ", ";
  }
  std::cout << std::endl;
}
