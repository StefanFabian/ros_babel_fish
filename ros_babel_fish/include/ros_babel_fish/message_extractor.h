// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_EXTRACTOR_H
#define ROS_BABEL_FISH_MESSAGE_EXTRACTOR_H

#include "ros_babel_fish/exceptions/invalid_location_exception.h"
#include "ros_babel_fish/babel_fish.h"
#include "ros_babel_fish/babel_fish_message.h"

namespace ros_babel_fish
{

namespace message_extraction
{
namespace MessageOffsetTypes
{
enum MessageOffsetType
{
  Fixed,
  String,
  Array,
  ArrayElement // If pointing to specific array element, will check if array has enough elements
};
}
using MessageOffsetType = MessageOffsetTypes::MessageOffsetType;

class MessageOffset
{
public:
  explicit MessageOffset( std::ptrdiff_t fixed_offset )
    : type_( MessageOffsetTypes::Fixed ), offset_( fixed_offset ), index_( 0 ) { }

  explicit MessageOffset( MessageOffsetType type, std::ptrdiff_t offset = 0,
                          std::vector<MessageOffset> array_offsets = {}, uint32_t index = 0 )
    : array_offsets_( std::move( array_offsets )), type_( type ), offset_( offset ), index_( index ) { }

  std::ptrdiff_t offset( const uint8_t *buffer, std::ptrdiff_t current_offset ) const;

  bool isFixed() const { return type_ == MessageOffsetTypes::Fixed; }

  std::ptrdiff_t fixedOffset() const { return offset_; }

  MessageOffsetType type() const { return type_; }

private:
  std::vector<MessageOffset> array_offsets_;
  MessageOffsetType type_;
  std::ptrdiff_t offset_;
  uint32_t index_;
};
}

class SubMessageLocation
{
public:
  SubMessageLocation();

  explicit SubMessageLocation( std::string root_type, MessageTemplate::ConstPtr msg_template,
                               std::vector<message_extraction::MessageOffset> offsets );

  std::ptrdiff_t calculateOffset( const IBabelFishMessage &msg ) const;

  const MessageTemplate::ConstPtr &messageTemplate() const { return msg_template_; }

  bool isValid() const { return msg_template_ != nullptr; }

  /*!
   * @return The type for which the sub-message location is valid.
   */
  const std::string &rootType() const { return root_type_; }

private:
  std::vector<message_extraction::MessageOffset> offsets_;
  MessageTemplate::ConstPtr msg_template_;
  std::string root_type_;
};

class MessageExtractor
{
public:
  explicit MessageExtractor( BabelFish &babel_fish );

  /*!
   * Finds the location of the sub message accessed by the given path.
   * If you have, for example, a PoseStamped message and are only interested in the positon which you would normally
   * in a translated message as @code message["pose"]["position"] @endcode
   * You can extract the location of the position and subsequently use the extractor to only evaluate that message.
   *
   * @param path The path to the submessage, e.g., ".pose.position". The first dot is optional.
   * @return An object that can be evaluated to obtain the location of the submessage
   */
  SubMessageLocation retrieveLocationForPath( const MessageTemplate::ConstPtr &msg_template, const std::string &path );

  SubMessageLocation retrieveLocationForPath( const std::string &base_msg, const std::string &path );

  SubMessageLocation retrieveLocationForPath( const IBabelFishMessage &msg, const std::string &path );

  TranslatedMessage::Ptr extractMessage( const IBabelFishMessage::ConstPtr &msg, const SubMessageLocation &location );

  Message::Ptr extractMessage( const IBabelFishMessage &msg, const SubMessageLocation &location );

  template<typename T>
  T extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location )
  {
    if ( msg.dataType() != location.rootType())
      throw InvalidLocationException(  "Message is of type '" + msg.dataType() +
                                      "' but location is for messages of type '" + location.rootType() + "'!" );
    if ( message_type_traits::message_type<T>::value != location.messageTemplate()->type )
      throw BabelFishException( "Tried to extract incompatible type from '" + msg.dataType() + "' message!" );
    std::ptrdiff_t offset = location.calculateOffset( msg );
    if ( offset == -1 ) throw BabelFishException( "Failed to locate submessage in '" + msg.dataType() + "' message!" );
    return *reinterpret_cast<const T *>(msg.buffer() + offset);
  }

  template<typename T>
  T extractValue( const IBabelFishMessage::ConstPtr &msg, const SubMessageLocation &location )
  {
    return extractValue<T>( *msg, location );
  }

private:
  BabelFish fish_;
};

template<>
std::string MessageExtractor::extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location );

template<>
ros::Time MessageExtractor::extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location );

template<>
ros::Duration MessageExtractor::extractValue( const IBabelFishMessage &msg, const SubMessageLocation &location );
} // ros_babel_fish

#endif //ROS_BABEL_FISH_MESSAGE_EXTRACTOR_H
