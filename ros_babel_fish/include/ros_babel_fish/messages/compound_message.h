// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_COMPOUND_MESSAGE_H
#define ROS_BABEL_FISH_COMPOUND_MESSAGE_H

#include "ros_babel_fish/generation/message_template.h"
#include "ros_babel_fish/message.h"

#include <vector>

namespace ros_babel_fish
{

class CompoundMessage : public Message
{

  explicit CompoundMessage( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream );

public:
  typedef std::shared_ptr<CompoundMessage> Ptr;
  typedef std::shared_ptr<const CompoundMessage> ConstPtr;

  static CompoundMessage *fromStream( const MessageTemplate::ConstPtr &msg_template, const uint8_t *stream,
                                      size_t stream_length, size_t &bytes_read );

  explicit CompoundMessage( const MessageTemplate::ConstPtr &msg_template );

  ~CompoundMessage() override;

  const std::string &datatype() const { return msg_template_->compound.datatype; }

  Message &operator[]( const std::string &key ) override;

  const Message &operator[]( const std::string &key ) const override;

  bool containsKey( const std::string &key ) const;

  const std::vector<std::string> &keys() const { return msg_template_->compound.names; }

  const std::vector<Message *> &values() const { return values_; }

  size_t _sizeInBytes() const override;

  bool isDetachedFromStream() const override;

  void detachFromStream() override;

  size_t writeToStream( uint8_t *stream ) const override;

  CompoundMessage &operator=( const CompoundMessage &other );

  Message *clone() const override;

protected:
  void assign( const Message &other ) override;

private:
  MessageTemplate::ConstPtr msg_template_;
  std::vector<Message *> values_;
};
} // ros_babel_fish

#endif //ROS_BABEL_FISH_COMPOUND_MESSAGE_H
