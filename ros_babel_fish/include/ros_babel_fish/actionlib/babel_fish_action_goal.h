// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_ACTION_GOAL_H
#define ROS_BABEL_FISH_BABEL_FISH_ACTION_GOAL_H

#include "ros_babel_fish/generation/providers/message_only_description_provider.h"
#include "ros_babel_fish/babel_fish_message.h"

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>

namespace ros_babel_fish
{

template<class ContainerAllocator>
struct BabelFishActionGoal_
{
  typedef BabelFishActionGoal_<ContainerAllocator> Type;

  BabelFishActionGoal_() = default;

  explicit BabelFishActionGoal_( const ContainerAllocator &allocator ) : header( allocator ), goal_id( allocator ) { }

  const std::string &md5Sum() const
  {
    if ( md5_.empty()) initMessageForGoal();
    return md5_;
  }

  const std::string &dataType() const
  {
    if ( datatype_.empty()) initMessageForGoal();
    return datatype_;
  }

  const std::string &definition() const
  {
    if ( definition_.empty()) initMessageForGoal();
    return definition_;
  }

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
                         action_goal_description->message_template->compound.names.end(), "goal" );
    if ( it == action_goal_description->message_template->compound.names.end())
      throw BabelFishException( "Did not find goal in ActionGoal message of type '" + datatype + "'!" );
    int goal_index = std::distance( action_goal_description->message_template->compound.names.begin(), it );
    MessageDescription::ConstPtr goal_description = provider.getMessageDescription(
      action_goal_description->message_template->compound.types[goal_index]->compound.datatype );
    goal.morph( goal_description );
  }

  void morph( const MessageDescription::ConstPtr &description )
  {
    morph( description->md5, description->datatype, description->message_definition );
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;
  typedef ::actionlib_msgs::GoalID_<ContainerAllocator> _goal_id_type;
  _goal_id_type goal_id;

  typedef BabelFishMessage _goal_type;
  _goal_type goal;

  typedef boost::shared_ptr<BabelFishActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<const BabelFishActionGoal_<ContainerAllocator> > ConstPtr;
private:

  void initMessageForGoal() const
  {
    if ( goal.dataType().empty()) return;

    // This method initializes the md5 sum, datatype and definition based on the values provided in the goal BabelFishMessage
    MessageOnlyDescriptionProvider provider;
    provider.registerMessageByDefinition( ros::message_traits::datatype<std_msgs::Header>(),
                                          ros::message_traits::definition<std_msgs::Header>());
    provider.registerMessageByDefinition( ros::message_traits::datatype<_goal_id_type>(),
                                          ros::message_traits::definition<_goal_id_type>());
    provider.registerMessageByDefinition( goal.dataType(), goal.definition());
    std::string datatype = goal.dataType();
    assert( datatype.substr( datatype.length() - 4 ) == "Goal" );
    datatype = datatype.substr( 0, datatype.length() - 4 ) + "ActionGoal";
    MessageDescription::ConstPtr description =
      provider.registerMessageBySpecification( datatype, std::string( "Header header\n" )
                                                         + ros::message_traits::datatype<_goal_id_type>() + " goal_id\n"
                                                         + goal.dataType() + " goal\n" );
    md5_ = description->md5;
    datatype_ = datatype;
    definition_ = description->message_definition;
  }

  mutable std::string md5_;
  mutable std::string datatype_;
  mutable std::string definition_;
  bool latched_ = false;
};

typedef BabelFishActionGoal_<std::allocator<void> > BabelFishActionGoal;
typedef typename BabelFishActionGoal::Ptr BabelFishActionGoalPtr;
typedef typename BabelFishActionGoal::ConstPtr BabelFishActionGoalConstPtr;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// Message and service traits for serialization API
namespace ros
{
namespace message_traits
{

template<class ContainerAllocator>
struct IsFixedSize<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > : FalseType
{
};
template<class ContainerAllocator>
struct IsFixedSize<const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > : FalseType
{
};

template<class ContainerAllocator>
struct IsMessage<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > : TrueType
{
};
template<class ContainerAllocator>
struct IsMessage<const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > : TrueType
{
};

template<class ContainerAllocator>
struct HasHeader<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > : TrueType
{
};
template<class ContainerAllocator>
struct HasHeader<const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > : TrueType
{
};

template<class ContainerAllocator>
struct MD5Sum<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> >
{
  static const char *
  value( const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> &m ) { return m.md5Sum().c_str(); }

  // A BabelFishActionGoal can be of any type
  static const char *value() { return "*"; }
};

template<class ContainerAllocator>
struct DataType<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> >
{
  static const char *
  value( const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> &m ) { return m.dataType().c_str(); }

  // A BabelFishActionGoal can be of any type
  static const char *value() { return "*"; }
};

template<class ContainerAllocator>
struct Definition<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> >
{
  static const char *
  value( const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> &m ) { return m.definition().c_str(); }

  // A BabelFishActionGoal can be of any type
  static const char *value() { return "*"; }
};
} // message_traits

namespace serialization
{

template<class ContainerAllocator>
struct Serializer<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> >
{
  template<typename Stream>
  inline static void write( Stream &stream, const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> &m )
  {
    stream.next( m.header );
    stream.next( m.goal_id );
    m.goal.write( stream );
  }

  template<typename Stream>
  inline static void read( Stream &stream, ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> &m )
  {
    stream.next( m.header );
    stream.next( m.goal_id );
    m.goal.read( stream );
  }

  inline static uint32_t serializedLength( const ::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> &m )
  {
    LStream stream;
    stream.next( m.header );
    stream.next( m.goal_id );
    return stream.getLength() + m.goal.size();
  }
};

template<class ContainerAllocator>
struct PreDeserialize<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> >
{
  static void notify( const PreDeserializeParams<::ros_babel_fish::BabelFishActionGoal_<ContainerAllocator> > &params )
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

#endif // ROS_BABEL_FISH_BABEL_FISH_ACTION_GOAL_H
