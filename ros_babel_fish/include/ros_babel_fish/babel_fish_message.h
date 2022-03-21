// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_MESSAGE_H
#define ROS_BABEL_FISH_BABEL_FISH_MESSAGE_H

#include "ros_babel_fish/message_description.h"

#include <ros/serialization.h>
#include <ros/service_callback_helper.h>
#include <ros/service_traits.h>

namespace ros_babel_fish
{

/*!
 * A read-only interface for BabelFishMessage.
 */
class IBabelFishMessage
{
public:
  typedef boost::shared_ptr<IBabelFishMessage> Ptr;
  typedef boost::shared_ptr<const IBabelFishMessage> ConstPtr;

  virtual ~IBabelFishMessage() = default;

  virtual const std::string &md5Sum() const = 0;

  virtual const std::string &dataType() const = 0;

  virtual const std::string &__getServerMD5Sum() const
  {
    static const std::string missing = "*";
    return missing;
  }

  virtual const std::string &__getServiceDatatype() const
  {
    static const std::string missing = "*";
    return missing;
  }

  virtual const std::string &definition() const = 0;

  virtual bool isLatched() const = 0;

  virtual uint32_t size() const = 0;

  virtual const uint8_t *buffer() const = 0;
};

/*!
 * A message that can store any type of message.
 * The message contents can be retrieved by translating it with a BabelFish.
 */
class BabelFishMessage : public IBabelFishMessage
{
public:
  typedef boost::shared_ptr<BabelFishMessage> Ptr;
  typedef boost::shared_ptr<const BabelFishMessage> ConstPtr;

  BabelFishMessage();

  BabelFishMessage( const BabelFishMessage &other );

  BabelFishMessage &operator=( const BabelFishMessage &other );

  ~BabelFishMessage() override;

  const std::string &md5Sum() const final;

  const std::string &dataType() const final;

  const std::string &__getServerMD5Sum() const final;

  const std::string &__getServiceDatatype() const final;

  const std::string &definition() const final;

  bool isLatched() const final;

  void morph( const std::string &md5sum, const std::string &datatype, const std::string &definition,
              bool latched = false, const std::string &server_md5sum = "*" );

  void morph( const MessageDescription::ConstPtr &description, const std::string &server_md5sum = "*" );

  /*!
   * Writes the serialized message to the given stream.
   * @tparam Stream A stream class providing an advance method that takes a uint32_t with the size and returns the
   *   pointer to a memory position that is big enough for the given amount of bytes
   * @param stream The stream the serialized message is written to
   */
  template<typename Stream>
  void write( Stream &stream ) const
  {
    if ( buffer_used_ > 0 )
      std::memcpy( stream.advance( buffer_used_ ), buffer_, buffer_used_ );
  }

  /*!
   * Reads a serialized message from a stream.
   * @tparam Stream A stream class providing a getData() method that returns a pointer to the data and a getLength()
   *   method that returns the size of the data in bytes.
   * @param stream The stream from which the serialized message is read.
   */
  template<typename Stream>
  void read( Stream &stream )
  {
    // Allocate buffer and copy data
    allocate( stream.getLength());
    std::memcpy( buffer_, stream.getData(), stream.getLength());
  }

  /*!
   * @return The size of the message which is the number of used bytes.
   */
  uint32_t size() const final;

  uint8_t *buffer() { return buffer_; }

  const uint8_t *buffer() const final { return buffer_; }

  void allocate( size_t size );

private:
  std::string md5_;
  std::string server_md5_;
  std::string datatype_;
  mutable std::string service_datatype_; // Generated on-demand
  std::string definition_;
  bool latched_;

  uint8_t *buffer_;
  uint32_t buffer_size_;
  uint32_t buffer_used_;
};

class BabelFishMessageException : public BabelFishException
{
public:
  explicit BabelFishMessageException( const std::string &msg ) : BabelFishException( msg ) { }
};
} // ros_babel_fish

// Message and service traits for serialization API
namespace ros
{
namespace message_traits
{

template<>
struct IsMessage<::ros_babel_fish::BabelFishMessage> : TrueType
{
};
template<>
struct IsMessage<const ::ros_babel_fish::BabelFishMessage> : TrueType
{
};

template<>
struct MD5Sum<::ros_babel_fish::BabelFishMessage>
{
  static const char *value( const ::ros_babel_fish::BabelFishMessage &m ) { return m.__getServerMD5Sum().c_str(); }

  // A BabelFishMessage can be of any type
  static const char *value() { return "*"; }
};

template<>
struct DataType<::ros_babel_fish::BabelFishMessage>
{
  static const char *value( const ::ros_babel_fish::BabelFishMessage &m ) { return m.dataType().c_str(); }

  // A BabelFishMessage can be of any type
  static const char *value() { return "*"; }
};

template<>
struct Definition<::ros_babel_fish::BabelFishMessage>
{
  static const char *value( const ::ros_babel_fish::BabelFishMessage &m ) { return m.definition().c_str(); }
};
} // message_traits

namespace service_traits
{
template<>
struct MD5Sum<::ros_babel_fish::BabelFishMessage>
{
  static const char *value( const ::ros_babel_fish::BabelFishMessage &m ) { return m.__getServerMD5Sum().c_str(); }

  static const char *value() { return "*"; }
};

template<>
struct DataType<::ros_babel_fish::BabelFishMessage>
{
  static const char *value( const ::ros_babel_fish::BabelFishMessage &m ) { return m.__getServiceDatatype().c_str(); }

  static const char *value() { return "*"; }
};
} // service_traits


namespace serialization
{

template<>
struct Serializer<::ros_babel_fish::BabelFishMessage>
{
  template<typename Stream>
  inline static void write( Stream &stream, const ::ros_babel_fish::BabelFishMessage &m )
  {
    m.write( stream );
  }

  template<typename Stream>
  inline static void read( Stream &stream, ::ros_babel_fish::BabelFishMessage &m )
  {
    m.read( stream );
  }

  inline static uint32_t serializedLength( const ::ros_babel_fish::BabelFishMessage &m )
  {
    return m.size();
  }
};


template<>
struct PreDeserialize<::ros_babel_fish::BabelFishMessage>
{
  static void notify( const PreDeserializeParams<::ros_babel_fish::BabelFishMessage> &params )
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

#endif //ROS_BABEL_FISH_BABEL_FISH_MESSAGE_H
