//
// Created by Stefan Fabian on 03.09.19.
//

#ifndef ROS_BABEL_FISH_MESSAGE_COMPARISON_H
#define ROS_BABEL_FISH_MESSAGE_COMPARISON_H

#include <ros_babel_fish/babel_fish_message.h>
#include <ros_babel_fish/message.h>
#include <ros_babel_fish/message_types.h>

#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros_babel_fish_test_msgs/TestArray.h>
#include <ros_babel_fish_test_msgs/TestMessage.h>
#include <ros/message_traits.h>

#define MESSAGE_TYPE_EQUAL( A, B ) MessageTypeEqual( A, B )
#define MESSAGE_CONTENT_EQUAL( A, B ) MessageContentEqual( A, B )

const char *MessageTypeName( ros_babel_fish::MessageType type )
{
  switch ( type )
  {
    case ros_babel_fish::MessageTypes::None:
      return "None";
    case ros_babel_fish::MessageTypes::Bool:
      return "Bool";
    case ros_babel_fish::MessageTypes::UInt8:
      return "UInt8";
    case ros_babel_fish::MessageTypes::UInt16:
      return "UInt16";
    case ros_babel_fish::MessageTypes::UInt32:
      return "UInt32";
    case ros_babel_fish::MessageTypes::UInt64:
      return "UInt64";
    case ros_babel_fish::MessageTypes::Int8:
      return "Int8";
    case ros_babel_fish::MessageTypes::Int16:
      return "Int16";
    case ros_babel_fish::MessageTypes::Int32:
      return "Int32";
    case ros_babel_fish::MessageTypes::Int64:
      return "Int64";
    case ros_babel_fish::MessageTypes::Float32:
      return "Float32";
    case ros_babel_fish::MessageTypes::Float64:
      return "Float64";
    case ros_babel_fish::MessageTypes::String:
      return "String";
    case ros_babel_fish::MessageTypes::Time:
      return "Time";
    case ros_babel_fish::MessageTypes::Duration:
      return "Duration";
    case ros_babel_fish::MessageTypes::Compound:
      return "Compound";
    case ros_babel_fish::MessageTypes::Array:
      return "Array";
  }
  return "Invalid";
}


template<typename MessageType>
::testing::AssertionResult MessageTypeEqual( const ros_babel_fish::BabelFishMessage &a, const MessageType & )
{
  using namespace ros::message_traits;
  if ( a.dataType() != DataType<MessageType>::value())
  {
    return ::testing::AssertionFailure() << "DataType mismatch!" << std::endl
                                         << "BF  DataType: " << a.dataType() << std::endl
                                         << "MSG DataType: " << DataType<MessageType>::value();
  }
  if ( a.definition() != Definition<MessageType>::value())
  {
    return ::testing::AssertionFailure() << "Definition mismatch!" << std::endl
                                         << "BF  Definition: " << std::endl << a.definition() << std::endl
                                         << "MSG Definition: " << std::endl << Definition<MessageType>::value();
  }
  if ( a.md5Sum() != MD5Sum<MessageType>::value())
  {
    return ::testing::AssertionFailure() << "MD5 mismatch!" << std::endl
                                         << "BF  MD5: " << std::endl << a.md5Sum() << std::endl
                                         << "MSG MD5: " << std::endl << MD5Sum<MessageType>::value();
  }

  return ::testing::AssertionSuccess();
}

template<typename MessageType>
::testing::AssertionResult MessageTypeEqual( const ros_babel_fish::BabelFishMessage &a,
                                             const std::shared_ptr<const MessageType> &b )
{
  return MessageTypeEqual( a, *b );
}

template<typename MessageType>
::testing::AssertionResult MessageTypeEqual( const ros_babel_fish::BabelFishMessage::ConstPtr &a, const MessageType &b )
{
  return MessageTypeEqual( *a, b );
}

template<typename MessageType>
::testing::AssertionResult MessageTypeEqual( const MessageType &b,
                                             const ros_babel_fish::BabelFishMessage &a )
{
  return MessageTypeEqual( a, b );
}

template<typename MessageType>
::testing::AssertionResult MessageTypeEqual( const MessageType &b, const ros_babel_fish::BabelFishMessage::ConstPtr &a )
{
  return MessageTypeEqual( *a, b );
}


/**********************************************************************************************************************/
/************************************************** Message Content ***************************************************/
/**********************************************************************************************************************/

template<typename T>
struct is_basic_type
{
  static constexpr bool value = std::is_fundamental<T>::value;
};

template<>
struct is_basic_type<ros::Time>
{
  static constexpr bool value = true;
};

template<>
struct is_basic_type<ros::Duration>
{
  static constexpr bool value = true;
};

template<>
struct is_basic_type<std::string>
{
  static constexpr bool value = true;
};

template<typename MessageType>
::testing::AssertionResult MessageContentEqual( const ros_babel_fish::Message &a, const MessageType &b,
                                                const std::string &path = "msg" );

template<typename T>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const T &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  using namespace ros_babel_fish;
  constexpr MessageType mt = message_type_traits::message_type<T>::value;
  if ( a.type() != mt )
  {
    result = ::testing::AssertionFailure() << "Datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: " << MessageTypeName( static_cast<MessageType>(a.type()))
                                           << std::endl
                                           << "MSG Datatype: " << typeid( b ).name();
    return false;
  }
  if ( a.as<ValueMessage<T>>().getValue() != b )
  {
    result = ::testing::AssertionFailure() << "Content at " << path << " doesn't match!" << std::endl
                                           << "BF  Content: " << a.as<ValueMessage<T>>().getValue() << std::endl
                                           << "MSG Content: " << b;
    return false;
  }
  return true;
}

template<typename T1, typename T2>
typename std::enable_if<!std::is_same<T1, ros_babel_fish::Message>::value, bool>::type
MessageContentEqualImpl( const T1 &a, const T2 &b, const std::string &path, ::testing::AssertionResult &result )
{
  if ( !std::is_same<T1, T2>::value )
  {
    result = ::testing::AssertionFailure() << "Datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: " << typeid( a ).name() << std::endl
                                           << "MSG Datatype: " << typeid( b ).name();
    return false;
  }
  if ( a != b )
  {
    result = ::testing::AssertionFailure() << "Content at " << path << " doesn't match!" << std::endl
                                           << "BF  Content: " << a << std::endl
                                           << "MSG Content: " << b;
    return false;
  }
  return true;
}

template<typename T>
typename std::enable_if<is_basic_type<T>::value, bool>::type sameType( const ros_babel_fish::Message &a,
                                                                       const std::string &path,
                                                                       ::testing::AssertionResult &result )
{
  using namespace ros_babel_fish;
  constexpr MessageType mt = message_type_traits::message_type<T>::value;
  if ( a.as<ArrayMessageBase>().elementType() != mt )
  {
    result = ::testing::AssertionFailure() << "Array element datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: "
                                           << MessageTypeName( a.as<ArrayMessageBase>().elementType()) << std::endl
                                           << "MSG Datatype: " << typeid( T ).name();
    return false;
  }
  return true;
}

template<typename T>
typename std::enable_if<!is_basic_type<T>::value, bool>::type sameType( const ros_babel_fish::Message &a,
                                                                        const std::string &path,
                                                                        ::testing::AssertionResult &result )
{
  using namespace ros_babel_fish;
  if ( a.as<ArrayMessageBase>().elementType() != MessageTypes::Compound )
  {
    result = ::testing::AssertionFailure() << "Array element datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: "
                                           << MessageTypeName( a.as<ArrayMessageBase>().elementType()) << std::endl
                                           << "MSG Datatype: " << typeid( T ).name();
    return false;
  }
  if ( ros::message_traits::DataType<T>::value() != a.as<CompoundArrayMessage>().elementDataType())
  {
    result = ::testing::AssertionFailure() << "Array element datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: " << a.as<CompoundArrayMessage>().elementDataType()
                                           << std::endl
                                           << "MSG Datatype: " << ros::message_traits::DataType<T>::value();
    return false;
  }
  return true;
}

template<typename T, typename A>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const std::vector<T, A> &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  using namespace ros_babel_fish;
  if ( a.type() != MessageTypes::Array )
  {
    result = ::testing::AssertionFailure() << "Datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: " << MessageTypeName( a.type())
                                           << std::endl
                                           << "MSG Datatype: Array";
    return false;
  }
  if ( !sameType<T>( a, path, result )) return false;
  auto &arr = a.as<ArrayMessage<typename std::conditional<is_basic_type<T>::value, T, Message>::type >>();
  if ( arr.length() != b.size())
  {
    result = ::testing::AssertionFailure() << "Array length at " << path << " doesn't match!" << std::endl
                                           << "BF  length: " << arr.length() << std::endl
                                           << "MSG length: " << b.size();
    return false;
  }
  for ( size_t i = 0; i < arr.length(); ++i )
  {
    if ( !(MessageContentEqualImpl( arr[i], b[i], path + "[" + std::to_string( i ) + "]", result ))) return false;
  }
  return true;
}

template<typename T, size_t L>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const boost::array<T, L> &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  using namespace ros_babel_fish;
  if ( a.type() != MessageTypes::Array )
  {
    result = ::testing::AssertionFailure() << "Datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: " << MessageTypeName( static_cast<MessageType>(a.type()))
                                           << std::endl
                                           << "MSG Datatype: Array";
    return false;
  }
  if ( !sameType<T>( a, path, result )) return false;
  auto &arr = a.as<ArrayMessage<typename std::conditional<is_basic_type<T>::value, T, Message>::type>>();
  if ( arr.length() != b.size())
  {
    result = ::testing::AssertionFailure() << "Array length at " << path << " doesn't match!" << std::endl
                                           << "BF  length: " << arr.length() << std::endl
                                           << "MSG length: " << b.size();
    return false;
  }
  for ( size_t i = 0; i < arr.length(); ++i )
  {
    if ( !MessageContentEqualImpl( arr[i], b[i], path + "[" + std::to_string( i ) + "]", result )) return false;
  }
  return true;
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const std::string &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  using namespace ros_babel_fish;
  constexpr MessageType mt = message_type_traits::message_type<std::string>::value;
  if ( a.type() != mt )
  {
    result = ::testing::AssertionFailure() << "Datatype at " << path << " doesn't match!" << std::endl
                                           << "BF  Datatype: " << MessageTypeName( static_cast<MessageType>(a.type()))
                                           << std::endl
                                           << "MSG Datatype: " << typeid( b ).name();
    return false;
  }
  if ( a.as<ValueMessage<std::string>>().getValue() != b )
  {
    result = ::testing::AssertionFailure() << "Content at " << path << " doesn't match!" << std::endl
                                           << "BF  Content: " << a.as<ValueMessage<std::string>>().getValue()
                                           << std::endl
                                           << "MSG Content: " << b;
    return false;
  }
  return true;
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const std_msgs::Header &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["frame_id"], b.frame_id, path + ".frame_id", result )) return false;
  if ( !MessageContentEqualImpl( a["seq"], b.seq, path + ".seq", result )) return false;
  return MessageContentEqualImpl( a["stamp"], b.stamp, path + ".stamp", result );
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const geometry_msgs::Point &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["x"], b.x, path + ".x", result )) return false;
  if ( !MessageContentEqualImpl( a["y"], b.y, path + ".y", result )) return false;
  return MessageContentEqualImpl( a["z"], b.z, path + ".z", result );
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const geometry_msgs::Quaternion &b,
                              const std::string &path, ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["w"], b.w, path + ".w", result )) return false;
  if ( !MessageContentEqualImpl( a["x"], b.x, path + ".x", result )) return false;
  if ( !MessageContentEqualImpl( a["y"], b.y, path + ".y", result )) return false;
  return MessageContentEqualImpl( a["z"], b.z, path + ".z", result );
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const geometry_msgs::Pose &b, const std::string &path,
                              ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["position"], b.position, path + ".position", result )) return false;
  return MessageContentEqualImpl( a["orientation"], b.orientation, path + ".orientation", result );;
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const geometry_msgs::PoseStamped &b,
                              const std::string &path, ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["header"], b.header, path + ".header", result )) return false;
  return MessageContentEqualImpl( a["pose"], b.pose, path + ".pose", result );
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const ros_babel_fish_test_msgs::TestMessage &b,
                              const std::string &path, ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["header"], b.header, path + ".header", result )) return false;
  if ( !MessageContentEqualImpl<bool>( a["b"], b.b, path + ".b", result )) return false;
  if ( !MessageContentEqualImpl( a["ui8"], b.ui8, path + ".ui8", result )) return false;
  if ( !MessageContentEqualImpl( a["ui16"], b.ui16, path + ".ui16", result )) return false;
  if ( !MessageContentEqualImpl( a["ui32"], b.ui32, path + ".ui32", result )) return false;
  if ( !MessageContentEqualImpl( a["ui64"], b.ui64, path + ".ui64", result )) return false;
  if ( !MessageContentEqualImpl( a["i8"], b.i8, path + ".i8", result )) return false;
  if ( !MessageContentEqualImpl( a["i16"], b.i16, path + ".i16", result )) return false;
  if ( !MessageContentEqualImpl( a["i32"], b.i32, path + ".i32", result )) return false;
  if ( !MessageContentEqualImpl( a["i64"], b.i64, path + ".i64", result )) return false;
  if ( !MessageContentEqualImpl( a["str"], b.str, path + ".str", result )) return false;
  if ( !MessageContentEqualImpl( a["t"], b.t, path + ".t", result )) return false;
  return MessageContentEqualImpl( a["d"], b.d, path + ".d", result );
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const ros_babel_fish_test_msgs::TestArray &b,
                              const std::string &path, ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["uint8s"], b.uint8s, path + ".uint8s", result )) return false;
  if ( !MessageContentEqualImpl( a["uint16s"], b.uint16s, path + ".uint16s", result )) return false;
  if ( !MessageContentEqualImpl( a["uint32s"], b.uint32s, path + ".uint32s", result )) return false;
  if ( !MessageContentEqualImpl( a["uint64s"], b.uint64s, path + ".uint64s", result )) return false;
  if ( !MessageContentEqualImpl( a["int8s"], b.int8s, path + ".int8s", result )) return false;
  if ( !MessageContentEqualImpl( a["int16s"], b.int16s, path + ".int16s", result )) return false;
  if ( !MessageContentEqualImpl( a["int32s"], b.int32s, path + ".int32s", result )) return false;
  if ( !MessageContentEqualImpl( a["int64s"], b.int64s, path + ".int64s", result )) return false;
  if ( !MessageContentEqualImpl( a["float32s"], b.float32s, path + ".float32s", result )) return false;
  if ( !MessageContentEqualImpl( a["float64s"], b.float64s, path + ".float64s", result )) return false;
  if ( !MessageContentEqualImpl( a["times"], b.times, path + ".times", result )) return false;
  if ( !MessageContentEqualImpl( a["durations"], b.durations, path + ".durations", result )) return false;
  if ( !MessageContentEqualImpl( a["strings"], b.strings, path + ".strings", result )) return false;
  if ( !MessageContentEqualImpl( a["subarrays"], b.subarrays, path + ".subarrays", result )) return false;
  return MessageContentEqualImpl( a["subarrays_fixed"], b.subarrays_fixed, path + ".subarrays_fixed", result );
}

template<>
bool MessageContentEqualImpl( const ros_babel_fish::Message &a, const ros_babel_fish_test_msgs::TestSubArray &b,
                              const std::string &path, ::testing::AssertionResult &result )
{
  if ( !MessageContentEqualImpl( a["ints"], b.ints, path + ".ints", result )) return false;
  if ( !MessageContentEqualImpl( a["strings"], b.strings, path + ".strings", result )) return false;
  return MessageContentEqualImpl( a["times"], b.times, path + ".times", result );
}

template<typename MessageType>
::testing::AssertionResult MessageContentEqual( const ros_babel_fish::Message &a, const MessageType &b,
                                                const std::string &path )
{
  try
  {
    ::testing::AssertionResult result = ::testing::AssertionSuccess();
    MessageContentEqualImpl( a, b, path, result );
    return result;
  }
  catch ( std::exception &ex )
  {
    return ::testing::AssertionFailure() << "Exception occured when comparing " << path << ": " << ex.what();
  }
}

template<typename MessageType>
::testing::AssertionResult MessageContentEqual( const ros_babel_fish::Message &a,
                                                const boost::shared_ptr<const MessageType> &b )
{
  return MessageContentEqual( a, *b );
}

template<typename MessageType>
::testing::AssertionResult MessageContentEqual( const ros_babel_fish::Message::ConstPtr &a, const MessageType &b )
{
  return MessageContentEqual( *a, b );
}

template<typename MessageType>
::testing::AssertionResult MessageContentEqual( const MessageType &b, const ros_babel_fish::Message &a )
{
  return MessageContentEqual( a, b );
}

template<typename MessageType>
::testing::AssertionResult
MessageContentEqual( const boost::shared_ptr<const MessageType> &b, const ros_babel_fish::Message &a )
{
  return MessageContentEqual( a, *b );
}

template<typename MessageType>
::testing::AssertionResult MessageContentEqual( const MessageType &b, const ros_babel_fish::Message::ConstPtr &a )
{
  return MessageContentEqual( *a, b );
}

#endif // ROS_BABEL_FISH_MESSAGE_COMPARISON_H
