//
// Created by Stefan Fabian on 03.03.20.
//

#include <actionlib/server/simple_action_server.h>

#include <ros_babel_fish_test_msgs/SimpleTestAction.h>

using namespace ros_babel_fish_test_msgs;

actionlib::SimpleActionServer<SimpleTestAction> *simple_server;

void simpleActionServerCallback( const SimpleTestGoalConstPtr &goal )
{
  for ( int i = 0; i < goal->goal; ++i )
  {
    if ( i >= 10 && i >= goal->goal / 2 )
    {
      SimpleTestResult result;
      result.result = i;
      simple_server->setAborted( result, "Failed to compute the answer to everything." );
      return;
    }
    SimpleTestFeedback feedback;
    feedback.feedback = i;
    simple_server->publishFeedback( feedback );
    usleep( 30000 );
    if ( simple_server->isPreemptRequested())
    {
      SimpleTestResult result;
      result.result = i;
      simple_server->setPreempted( result, "The answer to everything is:" );
      return;
    }
  }
  SimpleTestResult result;
  result.result = goal->goal - 1;
  simple_server->setSucceeded( result );
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "action_client_test_server" );
  ros::NodeHandle nh_;
  simple_server = new actionlib::SimpleActionServer<SimpleTestAction>( nh_, "simple", &simpleActionServerCallback,
                                                                       false );
  simple_server->start();

  ros::spin();
  return 0;
}
