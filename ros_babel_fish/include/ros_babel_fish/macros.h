// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MACROS_H
#define ROS_BABEL_FISH_MACROS_H

#define RBF_TEMPLATE_CALL( function, type, args... ) { \
  switch (type) \
  { \
    case MessageTypes::Compound: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Compound>::value>( args ); \
      break; \
    case MessageTypes::Array: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Array>::value>( args ); \
      break; \
    case MessageTypes::Bool: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Bool>::value>( args ); \
      break; \
    case MessageTypes::UInt8: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt8>::value>( args ); \
      break; \
    case MessageTypes::UInt16: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt16>::value>( args ); \
      break; \
    case MessageTypes::UInt32: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt32>::value>( args ); \
      break; \
    case MessageTypes::UInt64: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt64>::value>( args ); \
      break; \
    case MessageTypes::Int8: \
      function<::ros_babel_fish::message_type_traits::member_type<MessageTypes::Int8>::value>( args ); \
      break; \
    case MessageTypes::Int16: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Int16>::value>( args ); \
      break; \
    case MessageTypes::Int32: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Int32>::value>( args ); \
      break; \
    case MessageTypes::Int64: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Int64>::value>( args ); \
      break; \
    case MessageTypes::Float32: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Float32>::value>( args ); \
      break; \
    case MessageTypes::Float64: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Float64>::value>( args ); \
      break; \
    case MessageTypes::String: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::String>::value>( args ); \
      break; \
    case MessageTypes::Time: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Time>::value>( args ); \
      break; \
    case MessageTypes::Duration: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Duration>::value>( args ); \
      break; \
  } \
}

#define RBF_TEMPLATE_CALL_VALUE_TYPES( function, type, args... ) { \
  switch (type) \
  { \
    case MessageTypes::Bool: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Bool>::value>( args ); \
      break; \
    case MessageTypes::UInt8: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt8>::value>( args ); \
      break; \
    case MessageTypes::UInt16: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt16>::value>( args ); \
      break; \
    case MessageTypes::UInt32: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt32>::value>( args ); \
      break; \
    case MessageTypes::UInt64: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::UInt64>::value>( args ); \
      break; \
    case MessageTypes::Int8: \
      function<::ros_babel_fish::message_type_traits::member_type<MessageTypes::Int8>::value>( args ); \
      break; \
    case MessageTypes::Int16: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Int16>::value>( args ); \
      break; \
    case MessageTypes::Int32: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Int32>::value>( args ); \
      break; \
    case MessageTypes::Int64: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Int64>::value>( args ); \
      break; \
    case MessageTypes::Float32: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Float32>::value>( args ); \
      break; \
    case MessageTypes::Float64: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Float64>::value>( args ); \
      break; \
    case MessageTypes::String: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::String>::value>( args ); \
      break; \
    case MessageTypes::Time: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Time>::value>( args ); \
      break; \
    case MessageTypes::Duration: \
      function<::ros_babel_fish::message_type_traits::value_type<MessageTypes::Duration>::value>( args ); \
      break; \
  } \
}

#endif //ROS_BABEL_FISH_MACROS_H
