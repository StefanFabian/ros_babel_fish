// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_INVALID_MESSAGE_PATH_EXCEPTION_H
#define ROS_BABEL_FISH_INVALID_MESSAGE_PATH_EXCEPTION_H

#include "babel_fish_exception.h"

namespace ros_babel_fish
{

class InvalidMessagePathException : public BabelFishException
{
public:
  explicit InvalidMessagePathException( const std::string &msg ) : BabelFishException( msg ) { }
};
} // ros_babel_fish

#endif // ROS_BABEL_FISH_INVALID_MESSAGE_PATH_EXCEPTION_H
