// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/babel_fish_message.h"

namespace ros_babel_fish
{
BabelFishMessage::BabelFishMessage()
  : md5_( "*" ), server_md5_( "*" ), latched_( false ), buffer_( nullptr ), buffer_size_( 0 ), buffer_used_( 0 )
{
}

BabelFishMessage::BabelFishMessage( const BabelFishMessage &other )
  : md5_( other.md5_ ), server_md5_( other.server_md5_ ), datatype_( other.datatype_ )
    , service_datatype_( other.service_datatype_ ), definition_( other.definition_ ), latched_( other.latched_ )
    , buffer_size_( other.buffer_used_ ), buffer_used_( other.buffer_used_ )
{
  buffer_ = new uint8_t[other.buffer_used_];
  std::memcpy( buffer_, other.buffer_, other.buffer_used_ );
}

BabelFishMessage &BabelFishMessage::operator=( const BabelFishMessage &other )
{
  if ( this == &other ) return *this;

  md5_ = other.md5_;
  server_md5_ = other.server_md5_;
  datatype_ = other.datatype_;
  service_datatype_ = other.service_datatype_;
  definition_ = other.definition_;
  latched_ = other.latched_;
  allocate( other.buffer_used_ );
  std::memcpy( buffer_, other.buffer_, other.buffer_used_ );
  return *this;
}

BabelFishMessage::~BabelFishMessage()
{
  delete[] buffer_;

  buffer_ = nullptr;
  buffer_size_ = 0;
  buffer_used_ = 0;
}

const std::string &BabelFishMessage::md5Sum() const { return md5_; }

const std::string &BabelFishMessage::dataType() const { return datatype_; }

const std::string &BabelFishMessage::__getServerMD5Sum() const { return server_md5_; }

const std::string &BabelFishMessage::__getServiceDatatype() const
{
  if ( !service_datatype_.empty()) return service_datatype_;

  if ( strcmp( dataType().c_str() + dataType().length() - 7, "Request" ) == 0 )
  {
    service_datatype_ = dataType().substr( 0, dataType().length() - 7 );
    return service_datatype_;
  }
  if ( strcmp( dataType().c_str() + dataType().length() - 8, "Response" ) == 0 )
  {
    service_datatype_ = dataType().substr( 0, dataType().length() - 8 );
    return service_datatype_;
  }
  throw ros_babel_fish::BabelFishMessageException(
    "Tried to get service datatype for message that is not a service request or response! Datatype: " + dataType());
}

const std::string &BabelFishMessage::definition() const { return definition_; }

bool BabelFishMessage::isLatched() const { return latched_; }

void BabelFishMessage::morph( const std::string &md5sum, const std::string &datatype, const std::string &definition,
                              bool latched, const std::string &server_md5sum )
{
  md5_ = md5sum;
  server_md5_ = server_md5sum;
  datatype_ = datatype;
  definition_ = definition;
  latched_ = latched;
}

void BabelFishMessage::morph( const MessageDescription::ConstPtr &description, const std::string &server_md5sum )
{
  md5_ = description->md5;
  server_md5_ = server_md5sum;
  datatype_ = description->datatype;
  definition_ = description->message_definition;
  latched_ = false;
}

uint32_t BabelFishMessage::size() const
{
  return buffer_used_;
}

void BabelFishMessage::allocate( size_t size )
{
  buffer_used_ = size;
  // Only reallocate if necessary
  if ( buffer_size_ > size ) return;
  delete[] buffer_;
  buffer_ = new uint8_t[size];
  buffer_size_ = size;
}
}
