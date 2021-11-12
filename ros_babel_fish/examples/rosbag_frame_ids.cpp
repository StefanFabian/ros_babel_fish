// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/exceptions/invalid_message_path_exception.h>
#include <ros_babel_fish/generation/providers/message_only_description_provider.h>
#include <ros_babel_fish/message_extractor.h>
#include <ros_babel_fish/message_types.h>
#include <ros/serialization.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <unordered_set>

/*!
 * The following example demonstrates efficient processing of messages of unspecified type in a rosbag.
 * Given a rosbag as input, dumps the frame_id for each topic that has a header.
 */

using namespace ros_babel_fish;

/*!
 * Thin wrapper around rosbag::MessageInstance to make it compatible with BabelFishMessage.
 * This avoids the need for a conversion to a BabelFishMessage instance, which would mean always copying
 * the message body, definition, type name, etc for each individual message.
 * Instead, only the message body is lazily read from the bag when necessary.
 */
class RosbagBabelFishMessage : public IBabelFishMessage
{
public:
  // Always copy the MessageInstance since it pretty much only contains pointers.
  RosbagBabelFishMessage( rosbag::MessageInstance msg ) : mi_( std::move( msg )) { }

  const std::string &topic() const { return mi_.getTopic(); }

  const std::string &md5Sum() const final { return mi_.getMD5Sum(); }

  const std::string &dataType() const final { return mi_.getDataType(); }

  const std::string &definition() const final { return mi_.getMessageDefinition(); }

  bool isLatched() const final { return mi_.isLatching(); }

  std::string callerId() const { return mi_.getCallerId(); }

  uint32_t size() const final { return mi_.size(); }

  const uint8_t *buffer() const final
  {
    if ( buffer_.empty() and size() > 0 )
    {
      buffer_.resize( size());
      ros::serialization::OStream stream( buffer_.data(), buffer_.size());
      mi_.write( stream );
    }
    return buffer_.data();
  }

  template<class T>
  boost::shared_ptr<T> instantiate() const { mi_.template instantiate<T>(); }

private:
  const rosbag::MessageInstance mi_;
  mutable std::vector<uint8_t> buffer_;
};

int main( int argc, char **argv )
{
  if ( argc != 2 )
  {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: rosbag [BAG]" << std::endl;
    return 1;
  }

  auto description_provider = std::make_shared<MessageOnlyDescriptionProvider>();
  BabelFish fish( description_provider );
  MessageExtractor extractor( fish );

  rosbag::Bag bag( argv[1] );
  rosbag::View view( bag );

  // Get the set of relevant topics.
  // Also populates the MessageOnlyDescriptionProvider object with all message definitions in the given bag.
  std::unordered_set<std::string> topics;
  for ( const auto &c : view.getConnections())
  {
    auto desc = description_provider->getMessageDescription( c->datatype, c->md5sum, c->msg_def );
    const auto &field_names = desc->message_template->compound.names;
    if ( std::find( field_names.begin(), field_names.end(), "header" ) != field_names.end())
      topics.insert( c->topic );
  }

  for ( const rosbag::MessageInstance &mi: view )
  {
    if ( topics.empty())
      break;
    if ( topics.find( mi.getTopic()) == topics.end())
      continue;
    RosbagBabelFishMessage msg( mi );
    // Not that useful here, but the field location should usually be cached based on the message type.
    SubMessageLocation location = extractor.retrieveLocationForPath( msg, "header.frame_id" );
    auto frame_id = extractor.extractValue<std::string>( msg, location );
    std::cout << msg.topic() << ": " << frame_id << std::endl;
    topics.erase( msg.topic());
  }
  bag.close();
}
