// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_ACTION_FEEDBACK_H
#define ROS_BABEL_FISH_BABEL_FISH_ACTION_FEEDBACK_H

#include "ros_babel_fish/generation/providers/message_only_description_provider.h"
#include "ros_babel_fish/babel_fish_message.h"

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalStatus.h>

namespace ros_babel_fish
{

template<class ContainerAllocator>
struct BabelFishActionFeedback_
{
  typedef BabelFishActionFeedback_<ContainerAllocator> Type;

  BabelFishActionFeedback_() = default;

  explicit BabelFishActionFeedback_( const ContainerAllocator &allocator )
    : header( allocator ), status( allocator ) { }

  const std::string &md5Sum() const { return md5_; }

  const std::string &dataType() const { return datatype_; }

  const std::string &definition() const { return definition_; }

  bool isLatched() const { return latched_; }

  void morph( const std::string &md5sum, const std::string &datatype, const std::string &definition,
              bool latched = false )
  {
    md5_ = md5sum;
    datatype_ = datatype;
    definition_ = definition;
    latched_ = latched;

    MessageOnlyDescriptionProvider provider;
    MessageDescription::ConstPtr action_goal_description = provider.registerMessageByDefinition( datatype_,
                                                                                                 definition_ );
    assert( action_goal_description != nullptr );
    assert( action_goal_description->message_template != nullptr );
    assert( action_goal_description->message_template->type == MessageTypes::Compound );
    auto it = std::find( action_goal_description->message_template->compound.names.begin(),
                         action_goal_description->message_template->compound.names.end(), "feedback" );
    if ( it == action_goal_description->message_template->compound.names.end())
      throw BabelFishException( "Did not find feedback in ActionFeedback message of type '" + datatype + "'!" );
    int goal_index = std::distance( action_goal_description->message_template->compound.names.begin(), it );
    MessageDescription::ConstPtr goal_description = provider.getMessageDescription(
      action_goal_description->message_template->compound.types[goal_index]->compound.datatype );
    feedback.morph( goal_description );
  }

  void morph( const MessageDescription::ConstPtr &description )
  {
    morph( description->md5, description->datatype, description->message_definition );
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::actionlib_msgs::GoalStatus_<ContainerAllocator> _status_type;
  _status_type status;

  typedef BabelFishMessage _feedback_type;
  _feedback_type feedback;

  typedef boost::shared_ptr<BabelFishActionFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<const BabelFishActionFeedback_<ContainerAllocator> > ConstPtr;
private:
  std::string md5_;
  std::string datatype_;
  std::string definition_;
  bool latched_ = false;
};

typedef BabelFishActionFeedback_<std::allocator<void> > BabelFishActionFeedback;
typedef typename BabelFishActionFeedback::Ptr BabelFishActionFeedbackPtr;
typedef typename BabelFishActionFeedback::ConstPtr BabelFishActionFeedbackConstPtr;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// Message and service traits for serialization API
namespace ros
{
namespace message_traits
{

template<class ContainerAllocator>
struct IsFixedSize<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > : FalseType
{
};
template<class ContainerAllocator>
struct IsFixedSize<const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > : FalseType
{
};

template<class ContainerAllocator>
struct IsMessage<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > : TrueType
{
};
template<class ContainerAllocator>
struct IsMessage<const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > : TrueType
{
};

template<class ContainerAllocator>
struct HasHeader<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > : TrueType
{
};
template<class ContainerAllocator>
struct HasHeader<const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > : TrueType
{
};

template<class ContainerAllocator>
struct MD5Sum<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> >
{
  static const char *
  value( const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> &m ) { return m.md5Sum().c_str(); }

  // A BabelFishActionGoal can be of any type
  static const char *value() { return "*"; }
};

template<class ContainerAllocator>
struct DataType<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> >
{
  static const char *
  value( const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> &m ) { return m.dataType().c_str(); }

  // A BabelFishActionGoal can be of any type
  static const char *value() { return "*"; }
};

template<class ContainerAllocator>
struct Definition<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> >
{
  static const char *
  value( const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> &m ) { return m.definition().c_str(); }

  // A BabelFishActionGoal can be of any type
  static const char *value() { return "*"; }
};
} // message_traits

namespace serialization
{

template<class ContainerAllocator>
struct Serializer<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> >
{
  template<typename Stream>
  inline static void write( Stream &stream, const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> &m )
  {
    stream.next( m.header );
    stream.next( m.status );
    m.feedback.write( stream );
  }

  template<typename Stream>
  inline static void read( Stream &stream, ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> &m )
  {
    stream.next( m.header );
    stream.next( m.status );
    m.feedback.read( stream );
  }

  inline static uint32_t serializedLength( const ::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> &m )
  {
    LStream stream;
    stream.next( m.header );
    stream.next( m.status );
    return stream.getLength() + m.feedback.size();
  }
};

template<class ContainerAllocator>
struct PreDeserialize<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> >
{
  static void
  notify( const PreDeserializeParams<::ros_babel_fish::BabelFishActionFeedback_<ContainerAllocator> > &params )
  {
    std::string md5 = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def = (*params.connection_header)["message_definition"];
    std::string latching = (*params.connection_header)["latching"];

    params.message->morph( md5, datatype, msg_def, latching == "1" );
  }
};
} // serialization

} // ros

#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif //ROS_BABEL_FISH_BABEL_FISH_ACTION_FEEDBACK_H
