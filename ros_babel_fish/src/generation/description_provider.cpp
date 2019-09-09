// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/generation/description_provider.h"
#include "ros_babel_fish/message_types.h"

#include <regex>

namespace ros_babel_fish
{

DescriptionProvider::DescriptionProvider()
{
  initBuiltInTypes();
}

MessageDescription::ConstPtr DescriptionProvider::getMessageDescription( const std::string &type )
{
  // Check cache
  auto it = message_descriptions_.find( type );
  if ( it != message_descriptions_.end()) return it->second;

  MessageDescription::ConstPtr result = getMessageDescriptionImpl( type );
  if ( result == nullptr ) return result;

  it = message_descriptions_.find( type );
  if ( it == message_descriptions_.end())
  {
    message_descriptions_.insert( { type, result } );
  }
  return result;
}

MessageDescription::ConstPtr DescriptionProvider::getMessageDescription( const BabelFishMessage &msg )
{
  const std::string &type = msg.dataType();
  const std::string &md5 = msg.md5Sum();

  // Check cache
  auto it = message_descriptions_.find( type );
  if ( it != message_descriptions_.end())
  {
    if ( it->second->md5 != md5 )
    {
      ROS_WARN_ONCE( "BabelFish knows a message of the type '%s' but with a different md5 sum!", type.c_str());
    }
    return it->second;
  }

  const std::string &message_definition = msg.definition();
  std::string::size_type end_of_message = message_definition.find( "\n==================" );
  std::string specification = message_definition.substr( 0, end_of_message );

  MessageDescription::Ptr description = std::make_shared<MessageDescription>();
  description->datatype = type;
  description->message_definition = message_definition;
  description->md5 = md5;
  description->specification = specification;
  description->message_template = createTemplate( type, specification );
  if ( description->message_template == nullptr ) return nullptr;

  message_descriptions_.insert( { type, description } );
  return description;
}

ServiceDescription::ConstPtr DescriptionProvider::getServiceDescription( const std::string &type )
{
  // Check cache
  auto it = service_descriptions_.find( type );
  if ( it != service_descriptions_.end()) return it->second;

  ServiceDescription::ConstPtr result = getServiceDescriptionImpl( type );
  if ( result == nullptr ) return nullptr;

  it = service_descriptions_.find( type );
  if ( it == service_descriptions_.end())
  {
    service_descriptions_.insert( { type, result } );
  }
  return result;
}

MessageTemplate::Ptr DescriptionProvider::createTemplate( const std::string &type, const std::string &specification )
{
  size_t start = 0;
  size_t end;
  MessageTemplate::Ptr msg_template = std::make_shared<MessageTemplate>();
  msg_template->type = MessageTypes::Compound;
  msg_template->compound.datatype = type;
  std::regex constant_regex( R"(^\s*(\w+)\s+(\w+)\s*=\s*(.*)\s*$)" );
  std::regex field_regex( R"(^\s*(\w+\/?\w+)\s*(\[\d*\])?\s*(\w+)\s*)" );
  while ( true )
  {
    end = specification.find( '\n', start );
    std::smatch match;
    std::string::const_iterator first = specification.begin() + start;
    std::string::const_iterator last = end == std::string::npos ? specification.end() : specification.begin() + end;
    if ( std::regex_search( first, last, match, constant_regex ) && match.size() > 3 )
    {
      std::string name = match.str( 2 );
      Message::Ptr value;
      if ( match.str( 1 ) == "bool" || match.str( 1 ) == "uint8" )
      {
        value = std::make_shared<ValueMessage<uint8_t>>( static_cast<uint8_t>(std::stoi( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "int8" )
      {
        value = std::make_shared<ValueMessage<int8_t>>( static_cast<int8_t>(std::stoi( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "uint16" )
      {
        value = std::make_shared<ValueMessage<uint16_t>>( static_cast<uint16_t>(std::stoi( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "int16" )
      {
        value = std::make_shared<ValueMessage<int16_t>>( static_cast<int16_t>(std::stoi( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "uint32" )
      {
        value = std::make_shared<ValueMessage<uint32_t>>( static_cast<uint32_t>(std::stoul( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "int32" )
      {
        value = std::make_shared<ValueMessage<int32_t>>( static_cast<int32_t>(std::stoi( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "uint64" )
      {
        value = std::make_shared<ValueMessage<uint64_t>>( static_cast<uint64_t>(std::stoul( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "int64" )
      {
        value = std::make_shared<ValueMessage<int64_t>>( static_cast<int64_t>(std::stol( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "float32" )
      {
        value = std::make_shared<ValueMessage<float>>( static_cast<float>(std::stof( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "float64" )
      {
        value = std::make_shared<ValueMessage<double>>( static_cast<double>(std::stod( match.str( 3 ))));
      }
      else if ( match.str( 1 ) == "string" )
      {
        value = std::make_shared<ValueMessage<std::string>>( match.str( 3 ));
      }
      msg_template->constants.insert( { name, value } );
    }
    else if ( std::regex_search( first, last, match, field_regex ) && match.size() > 3 )
    {
      msg_template->compound.names.push_back( match.str( 3 ));
      std::string type_name = match.str( 1 );
      if ( type_name == "Header" )
      {
        type_name = "std_msgs/Header";
      }
      else if ( builtin_types_.find( type_name ) == builtin_types_.end() && type_name.find( '/' ) == std::string::npos )
      {
        // Type is relative
        std::string package = type.substr( 0, type.find( '/' ));
        type_name = package.append( "/" ).append( type_name );
      }
      MessageDescription::ConstPtr sub_message_description = getMessageDescription( type_name );
      if ( sub_message_description == nullptr ) return nullptr;
      MessageTemplate::ConstPtr sub_template = sub_message_description->message_template;
      // Check if the declared type is an array by checking if the array specifier group is empty
      if ( match.str( 2 ).empty())
      {
        msg_template->compound.types.push_back( sub_template );
      }
      else // It's an array
      {
        MessageTemplate::Ptr array_template = std::make_shared<MessageTemplate>();
        array_template->type = MessageTypes::Array;
        std::string array_specifier = match.str( 2 );
        ssize_t length = -1;
        if ( array_specifier.size() > 2 )
          length = std::stol( array_specifier.substr( 1, array_specifier.size() - 2 ));
        array_template->array.length = length;
        array_template->array.element_type = sub_template->type;
        array_template->array.element_template = sub_template;
        msg_template->compound.types.push_back( array_template );
      }
    }
    if ( end == std::string::npos ) break;
    start = end + 1;
  }
  return msg_template;
}

MessageDescription::ConstPtr DescriptionProvider::registerMessage( const std::string &type,
                                                                   const std::string &definition,
                                                                   const std::string &md5,
                                                                   const std::string &specification )
{
  auto it = message_descriptions_.find( type );
  if ( it != message_descriptions_.end()) return it->second;
  MessageDescription::Ptr description = std::make_shared<MessageDescription>();
  description->datatype = type;
  description->message_definition = definition;
  description->md5 = md5;
  description->specification = specification;
  description->message_template = createTemplate( type, specification );
  if ( description->message_template == nullptr ) return nullptr;

  message_descriptions_.insert( { type, description } );
  return description;
}

ServiceDescription::ConstPtr DescriptionProvider::registerService( const std::string &type, const std::string &md5,
                                                                   const std::string &specification,
                                                                   const std::string &req_message_definition,
                                                                   const std::string &req_md5,
                                                                   const std::string &req_specification,
                                                                   const std::string &resp_message_definition,
                                                                   const std::string &resp_md5,
                                                                   const std::string &resp_specification )
{
  auto it = service_descriptions_.find( type );
  if ( it != service_descriptions_.end()) return it->second;
  ServiceDescription::Ptr description = std::make_shared<ServiceDescription>();
  description->datatype = type;
  description->md5 = md5;
  description->specification = specification;

  description->request = registerMessage( type + "Request", req_message_definition, req_md5, req_specification );

  description->response = registerMessage( type + "Response", resp_message_definition, resp_md5, resp_specification );

  service_descriptions_.insert( { type, description } );
  return description;
}

namespace
{
std::pair<std::string, MessageDescription::Ptr> makeDescription( const std::string &name, MessageType type )
{
  MessageTemplate::Ptr msg_template = std::make_shared<MessageTemplate>();
  msg_template->type = type;
  msg_template->compound.datatype = name;

  MessageDescription::Ptr description = std::make_shared<MessageDescription>();
  description->datatype = name;
  description->message_template = msg_template;
  return { name, description };
}
}

void DescriptionProvider::initBuiltInTypes()
{
  builtin_types_.insert( "bool" );
  message_descriptions_.insert( makeDescription( "bool", MessageTypes::Bool ));

  builtin_types_.insert( "uint8" );
  message_descriptions_.insert( makeDescription( "uint8", MessageTypes::UInt8 ));

  builtin_types_.insert( "int8" );
  message_descriptions_.insert( makeDescription( "int8", MessageTypes::Int8 ));

  builtin_types_.insert( "uint16" );
  message_descriptions_.insert( makeDescription( "uint16", MessageTypes::UInt16 ));

  builtin_types_.insert( "int16" );
  message_descriptions_.insert( makeDescription( "int16", MessageTypes::Int16 ));

  builtin_types_.insert( "uint32" );
  message_descriptions_.insert( makeDescription( "uint32", MessageTypes::UInt32 ));

  builtin_types_.insert( "int32" );
  message_descriptions_.insert( makeDescription( "int32", MessageTypes::Int32 ));

  builtin_types_.insert( "uint64" );
  message_descriptions_.insert( makeDescription( "uint64", MessageTypes::UInt64 ));

  builtin_types_.insert( "int64" );
  message_descriptions_.insert( makeDescription( "int64", MessageTypes::Int64 ));

  builtin_types_.insert( "float32" );
  message_descriptions_.insert( makeDescription( "float32", MessageTypes::Float32 ));

  builtin_types_.insert( "float64" );
  message_descriptions_.insert( makeDescription( "float64", MessageTypes::Float64 ));

  builtin_types_.insert( "string" );
  message_descriptions_.insert( makeDescription( "string", MessageTypes::String ));

  builtin_types_.insert( "time" );
  message_descriptions_.insert( makeDescription( "time", MessageTypes::Time ));

  builtin_types_.insert( "duration" );
  message_descriptions_.insert( makeDescription( "duration", MessageTypes::Duration ));
}
} // ros_babel_fish
