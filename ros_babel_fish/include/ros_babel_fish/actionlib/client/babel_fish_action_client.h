/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ROS_BABEL_FISH_BABEL_FISH_ACTION_CLIENT_H
#define ROS_BABEL_FISH_BABEL_FISH_ACTION_CLIENT_H

#include "ros_babel_fish/actionlib/babel_fish_action.h"

#include <actionlib/client/action_client.h>

namespace actionlib
{

template<class ContainerAllocator>
class ActionClient<::ros_babel_fish::BabelFishAction_<ContainerAllocator> >
{
public:
  typedef ::ros_babel_fish::BabelFishAction_<ContainerAllocator> ActionSpec;
  typedef ClientGoalHandle<ActionSpec> GoalHandle;

private:
  ACTION_DEFINITION( ActionSpec );
  typedef ActionClient<ActionSpec> ActionClientT;
  typedef boost::function<void( GoalHandle )> TransitionCallback;
  typedef boost::function<void( GoalHandle, const FeedbackConstPtr & )> FeedbackCallback;
  typedef boost::function<void( const ActionGoalConstPtr )> SendGoalFunc;

public:
  ActionClient( const ::ros_babel_fish::MessageDescription::ConstPtr &goal_description, const std::string &name,
                ros::CallbackQueueInterface *queue = NULL )
    : n_( name ), guard_( new DestructionGuard()), manager_( guard_ )
  {
    initClient( goal_description, queue );
  }

  ActionClient( const ros::NodeHandle &n, const ::ros_babel_fish::MessageDescription::ConstPtr &goal_description,
                const std::string &name, ros::CallbackQueueInterface *queue = NULL )
    : n_( n, name ), guard_( new DestructionGuard()), manager_( guard_ )
  {
    initClient( goal_description, queue );
  }

  ~ActionClient()
  {
    ROS_DEBUG_NAMED( "actionlib", "ActionClient: Waiting for destruction guard to clean up" );
    guard_->destruct();
    ROS_DEBUG_NAMED( "actionlib", "ActionClient: destruction guard destruct() done" );
  }


  GoalHandle sendGoal( const Goal &goal,
                       TransitionCallback transition_cb = TransitionCallback(),
                       FeedbackCallback feedback_cb = FeedbackCallback())
  {
    ROS_DEBUG_NAMED( "actionlib", "about to start initGoal()" );
    GoalHandle gh = manager_.initGoal( goal, transition_cb, feedback_cb );
    ROS_DEBUG_NAMED( "actionlib", "Done with initGoal()" );

    return gh;
  }

  void cancelAllGoals()
  {
    actionlib_msgs::GoalID cancel_msg;
    // CancelAll policy encoded by stamp=0, id=0
    cancel_msg.stamp = ros::Time( 0, 0 );
    cancel_msg.id = "";
    cancel_pub_.publish( cancel_msg );
  }

  void cancelGoalsAtAndBeforeTime( const ros::Time &time )
  {
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.stamp = time;
    cancel_msg.id = "";
    cancel_pub_.publish( cancel_msg );
  }

  bool waitForActionServerToStart( const ros::Duration &timeout = ros::Duration( 0, 0 ))
  {
    // if ros::Time::isSimTime(), then wait for it to become valid
    if ( !ros::Time::waitForValid( ros::WallDuration( timeout.sec, timeout.nsec )))
    {
      return false;
    }

    if ( connection_monitor_ )
    {
      return connection_monitor_->waitForActionServerToStart( timeout, n_ );
    }
    else
    {
      return false;
    }
  }

  bool isServerConnected()
  {
    return connection_monitor_->isServerConnected();
  }

private:
  ros::NodeHandle n_;

  boost::shared_ptr<DestructionGuard> guard_;
  GoalManager<ActionSpec> manager_;

  ros::Subscriber result_sub_;
  ros::Subscriber feedback_sub_;

  boost::shared_ptr<ConnectionMonitor> connection_monitor_;   // Have to destroy subscribers and publishers before the connection_monitor_, since we call callbacks in the connection_monitor_

  ros::Publisher goal_pub_;
  ros::Publisher cancel_pub_;
  ros::Subscriber status_sub_;

  void sendGoalFunc( const ActionGoalConstPtr &action_goal )
  {
    goal_pub_.publish( action_goal );
  }

  void sendCancelFunc( const actionlib_msgs::GoalID &cancel_msg )
  {
    cancel_pub_.publish( cancel_msg );
  }

  void initClient( const ::ros_babel_fish::MessageDescription::ConstPtr &description,
                   ros::CallbackQueueInterface *queue )
  {
    ros::Time::waitForValid();
    // read parameters indicating publish/subscribe queue sizes
    int pub_queue_size;
    int sub_queue_size;
    n_.param( "actionlib_client_pub_queue_size", pub_queue_size, 0 );
    n_.param( "actionlib_client_sub_queue_size", sub_queue_size, 0 );
    if ( pub_queue_size < 0 ) { pub_queue_size = 0; }
    if ( sub_queue_size < 0 ) { sub_queue_size = 0; }

    status_sub_ = queue_subscribe( "status", static_cast<uint32_t>(sub_queue_size),
                                   &ActionClientT::statusCb, this, queue );
    feedback_sub_ = queue_subscribe( "feedback", static_cast<uint32_t>(sub_queue_size),
                                     &ActionClientT::feedbackCb, this, queue );
    result_sub_ = queue_subscribe( "result", static_cast<uint32_t>(sub_queue_size),
                                   &ActionClientT::resultCb, this, queue );

    connection_monitor_.reset( new ConnectionMonitor( feedback_sub_, result_sub_ ));

    // Start publishers and subscribers
    goal_pub_ = advertiseGoal( description, "goal", static_cast<uint32_t>(pub_queue_size),
                               boost::bind( &ConnectionMonitor::goalConnectCallback, connection_monitor_,
                                            _1 ),
                               boost::bind( &ConnectionMonitor::goalDisconnectCallback,
                                            connection_monitor_, _1 ),
                               queue );
    cancel_pub_ =
      queue_advertise<actionlib_msgs::GoalID>( "cancel", static_cast<uint32_t>(pub_queue_size),
                                               boost::bind( &ConnectionMonitor::cancelConnectCallback,
                                                            connection_monitor_, _1 ),
                                               boost::bind( &ConnectionMonitor::cancelDisconnectCallback,
                                                            connection_monitor_, _1 ),
                                               queue );

    manager_.registerSendGoalFunc( boost::bind( &ActionClientT::sendGoalFunc, this, _1 ));
    manager_.registerCancelFunc( boost::bind( &ActionClientT::sendCancelFunc, this, _1 ));
  }

  ros::Publisher advertiseGoal( const ::ros_babel_fish::MessageDescription::ConstPtr &description,
                                const std::string &topic, uint32_t queue_size,
                                const ros::SubscriberStatusCallback &connect_cb,
                                const ros::SubscriberStatusCallback &disconnect_cb,
                                ros::CallbackQueueInterface *queue )
  {
    ros::AdvertiseOptions opts( topic, queue_size, description->md5, description->datatype,
                                description->message_definition, connect_cb, disconnect_cb );
    opts.tracked_object = ros::VoidPtr();
    opts.latch = false;
    opts.callback_queue = queue;
    return n_.advertise( opts );
  }

  template<class M>
  ros::Publisher queue_advertise( const std::string &topic, uint32_t queue_size,
                                  const ros::SubscriberStatusCallback &connect_cb,
                                  const ros::SubscriberStatusCallback &disconnect_cb,
                                  ros::CallbackQueueInterface *queue )
  {
    ros::AdvertiseOptions ops;
    ops.init<M>( topic, queue_size, connect_cb, disconnect_cb );
    ops.tracked_object = ros::VoidPtr();
    ops.latch = false;
    ops.callback_queue = queue;
    return n_.advertise( ops );
  }

  template<class M, class T>
  ros::Subscriber queue_subscribe( const std::string &topic, uint32_t queue_size, void (T::* fp)(
    const ros::MessageEvent<M const> & ), T *obj, ros::CallbackQueueInterface *queue )
  {
    ros::SubscribeOptions ops;
    ops.callback_queue = queue;
    ops.topic = topic;
    ops.queue_size = queue_size;
    ops.md5sum = ros::message_traits::md5sum<M>();
    ops.datatype = ros::message_traits::datatype<M>();
    ops.helper = ros::SubscriptionCallbackHelperPtr(
      new ros::SubscriptionCallbackHelperT<const ros::MessageEvent<M const> &>(
        boost::bind( fp, obj, _1 )
      )
    );
    return n_.subscribe( ops );
  }

  void statusCb( const ros::MessageEvent<actionlib_msgs::GoalStatusArray const> &status_array_event )
  {
    ROS_DEBUG_NAMED( "actionlib", "Getting status over the wire." );
    if ( connection_monitor_ )
    {
      connection_monitor_->processStatus(
        status_array_event.getConstMessage(), status_array_event.getPublisherName());
    }
    manager_.updateStatuses( status_array_event.getConstMessage());
  }

  void feedbackCb( const ros::MessageEvent<ActionFeedback const> &action_feedback )
  {
    manager_.updateFeedbacks( action_feedback.getConstMessage());
  }

  void resultCb( const ros::MessageEvent<ActionResult const> &action_result )
  {
    manager_.updateResults( action_result.getConstMessage());
  }
};
}

#endif //ROS_BABEL_FISH_BABEL_FISH_ACTION_CLIENT_H
