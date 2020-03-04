// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_ACTION_H
#define ROS_BABEL_FISH_BABEL_FISH_ACTION_H

#include "ros_babel_fish/actionlib/babel_fish_action_feedback.h"
#include "ros_babel_fish/actionlib/babel_fish_action_goal.h"
#include "ros_babel_fish/actionlib/babel_fish_action_result.h"

namespace ros_babel_fish
{

template<class ContainerAllocator>
struct BabelFishAction_
{
  typedef BabelFishAction_<ContainerAllocator> Type;

  BabelFishAction_() = default;

  explicit BabelFishAction_( const ContainerAllocator &allocator ) : action_goal( allocator ) { }

  typedef BabelFishActionGoal_<ContainerAllocator> _action_goal_type;
  _action_goal_type action_goal;
  typedef BabelFishActionResult_<ContainerAllocator> _action_result_type;
  _action_result_type action_result;
  typedef BabelFishActionFeedback_<ContainerAllocator> _action_feedback_type;
  _action_feedback_type action_feedback;

  typedef boost::shared_ptr<BabelFishActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<const BabelFishActionGoal_<ContainerAllocator> > ConstPtr;
};

typedef BabelFishAction_<std::allocator<void> > BabelFishAction;
}

// Include specialization for ActionClient and SimpleActionClient
#include "ros_babel_fish/actionlib/client/simple_action_client.h"

#endif //ROS_BABEL_FISH_BABEL_FISH_ACTION_H
