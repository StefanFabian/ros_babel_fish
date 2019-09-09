// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_COMPOUND_MESSAGE_H
#define ROS_BABEL_FISH_COMPOUND_MESSAGE_H

#include "ros_babel_fish/message.h"

#include <vector>

namespace ros_babel_fish
{

class CompoundMessage : public Message
{
public:
  typedef std::shared_ptr<CompoundMessage> Ptr;
  typedef std::shared_ptr<const CompoundMessage> ConstPtr;

  explicit CompoundMessage( std::string datatype, const uint8_t *stream = nullptr );

  ~CompoundMessage() override;

  const std::string &datatype() const { return datatype_; }

  Message &operator[]( const std::string &key ) override;

  const Message &operator[]( const std::string &key ) const override;

  bool containsKey( const std::string &key ) const;

  const std::vector<std::string> &keys() const { return keys_; }

  const std::vector<Message *> &values() const { return values_; }

  void insert( const std::string &key, Message *value );

  size_t size() const override;

  bool isDetachedFromStream() const override;

  void detachFromStream() override;

  size_t writeToStream( uint8_t *stream ) const override;

  CompoundMessage &operator=( const CompoundMessage &other );

  Message *clone() const override;

protected:
  void assign( const Message &other ) override;

private:
  std::string datatype_;
  std::vector<std::string> keys_;
  std::vector<Message *> values_;
};
} // ros_babel_fish

#endif //ROS_BABEL_FISH_COMPOUND_MESSAGE_H
