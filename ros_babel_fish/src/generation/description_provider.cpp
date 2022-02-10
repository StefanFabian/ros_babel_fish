// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/generation/description_provider.h"
#include "ros_babel_fish/message_types.h"

#include <openssl/md5.h>
#include <regex>
#include <ros_babel_fish/generation/message_template.h>

namespace ros_babel_fish
{

namespace
{
std::string md5ToString( const unsigned char md5[MD5_DIGEST_LENGTH] )
{
  std::stringstream md5string;
  md5string << std::hex << std::setfill( '0' );
  for ( unsigned int i = 0; i < MD5_DIGEST_LENGTH; ++i )
    md5string << std::setw( 2 ) << (int) md5[i];
  return md5string.str();
}
}

DescriptionProvider::DescriptionProvider()
{
  initBuiltInTypes();
}

bool DescriptionProvider::isBuiltIn( const std::string &type ) const
{
  return builtin_types_.find( type ) != builtin_types_.end();
}

MessageDescription::ConstPtr DescriptionProvider::getMessageDescription( const std::string &type )
{
  // Check cache
  auto it = message_descriptions_.find( type );
  if ( it != message_descriptions_.end()) return it->second;

  return getMessageDescriptionImpl( type );
}

MessageDescription::ConstPtr DescriptionProvider::getMessageDescription( const IBabelFishMessage &msg )
{
  return getMessageDescription( msg.dataType(), msg.md5Sum(), msg.definition());
}

MessageDescription::ConstPtr
DescriptionProvider::getMessageDescription( const std::string &type, const std::string &md5,
                                            const std::string &definition )
{
  // Check cache
  auto it = message_descriptions_.find( type );
  if ( it != message_descriptions_.end())
  {
    if ( it->second->md5 != md5 )
    {
      throw BabelFishException( "Message '" + type +"' found but MD5 sum differed!\n" +
                                md5 + " (provided) vs " + it->second->md5 + " (cached)." );
    }
    return it->second;
  }

  return getMessageDescriptionImpl( type, definition );
}

ServiceDescription::ConstPtr DescriptionProvider::getServiceDescription( const std::string &type )
{
  // Check cache
  auto it = service_descriptions_.find( type );
  if ( it != service_descriptions_.end()) return it->second;

  return getServiceDescriptionImpl( type );
}

MessageDescription::ConstPtr DescriptionProvider::getMessageDescriptionImpl( const std::string &type,
                                                                             const std::string &definition )
{
  // This will split the message definition and register all of the embedded message types, so look ups can be avoided
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );

  std::string::size_type start = definition.find( "\n===" );
  std::string::size_type end;
  if ( start != std::string::npos )
  {
    // Message contains dependencies, collect specs and dependencies for each spec
    std::vector<std::string> types;
    std::vector<std::string> specs;
    std::vector<std::vector<std::string>> dependencies;

    static std::regex field_type_regex( R"(^\s*(\w+(?:/\w+)?))" );
    std::smatch match;
    std::string buffer;
    std::string curr_package;
    buffer.reserve( 2048 );
    start = definition.find( '\n', start + 3 ) + 1;
    dependencies.emplace_back();
    bool msg_line = true;
    while ( true )
    {
      end = definition.find( '\n', start );
      if ( definition[start] == '=' && definition[start + 1] == '=' && definition[start + 2] == '=' )
      {
        if ( !buffer.empty()) buffer.pop_back();
        specs.push_back( buffer );
        buffer.clear();
        dependencies.emplace_back();
        start = end + 1;
        msg_line = true;
        continue;
      }

      if ( msg_line )
      {
        start += 5;
        std::string msg_type = definition.substr( start, end - start );
        if ( msg_type == "Header" ) msg_type = "std_msgs/Header";
        if ( msg_type.find( '/' ) == std::string::npos ) msg_type.insert( 0, package + '/' );
        curr_package = msg_type.substr( 0, msg_type.find( '/' ));
        types.push_back( msg_type );
        start = end + 1;
        msg_line = false;
        continue;
      }

      std::string::const_iterator first = definition.begin() + start;
      std::string::const_iterator last = end == std::string::npos ? definition.end() : definition.begin() + end;
      if ( std::regex_search( first, last, match, field_type_regex ) && match.size() == 2 )
      {
        std::string field_type = match.str( 1 );
        if ( !isBuiltIn( field_type ))
        {
          if ( field_type == "Header" ) field_type = "std_msgs/Header";
          if ( field_type.find( '/' ) == std::string::npos ) field_type.insert( 0, curr_package + '/' );
          bool found = false;
          for ( auto &dep : dependencies[specs.size()] )
          {
            if ( dep != field_type ) continue;
            found = true;
            break;
          }
          if ( !found )
          {
            dependencies[specs.size()].push_back( field_type );
          }
        }
      }

      if ( end == std::string::npos )
      {
        buffer += definition.substr( start );
        break;
      }
      buffer += definition.substr( start, end - start + 1 );
      start = end + 1;
    }
    specs.push_back( buffer );

    // Register all message dependencies
    bool registered[specs.size()];
    std::fill_n( registered, specs.size(), false );
    bool new_registered = true;
    while ( new_registered )
    {
      new_registered = false;
      for ( size_t i = 0; i < specs.size(); ++i )
      {
        if ( registered[i] ) continue;
        new_registered = true;
        registered[i] = true;
        if ( msg_specs_.find( types[i] ) != msg_specs_.end()) continue;
        // Check if all dependencies have been registered
        bool all_deps = true;
        for ( auto &dep : dependencies[i] )
        {
          if ( msg_specs_.find( dep ) != msg_specs_.end()) continue;
          all_deps = false;
          registered[i] = false;
        }
        if ( !all_deps ) continue;
        MessageSpec spec = createSpec( types[i], types[i].substr( 0, types[i].find( '/' )), specs[i] );
        if ( spec.md5.empty())
        {
          ROS_DEBUG_NAMED( "RosBabelFish", "Failed to compute MD5 for message '%s'!", type.c_str());
          return nullptr;
        }
        registerMessage( spec, computeFullText( spec ));
      }
    }
  }

  // Now register the message
  end = definition.find( "\n===" );
  MessageSpec spec = createSpec( type, package, definition.substr( 0, end ));
  if ( spec.md5.empty())
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Failed to compute MD5 for message '%s'!", type.c_str());
    return nullptr;
  }
  return registerMessage( spec, computeFullText( spec ));
}

MessageDescription::ConstPtr DescriptionProvider::getMessageDescriptionImpl( const IBabelFishMessage &msg )
{
  return getMessageDescriptionImpl( msg.dataType(), msg.definition());
}

MessageTemplate::Ptr DescriptionProvider::createTemplate( const MessageSpec &spec )
{
  MessageTemplate::Ptr msg_template = std::make_shared<MessageTemplate>();
  msg_template->type = MessageTypes::Compound;
  msg_template->compound.datatype = spec.name;
  // Initialize constants
  for ( auto &constant : spec.constants )
  {
    Message::Ptr value;
    if ( constant.type == "uint8" || constant.type == "char" )
    {
      value = std::make_shared<ValueMessage<uint8_t>>( static_cast<uint8_t>(std::stoi( constant.val )));
    }
    else if ( constant.type == "bool" )
    {
      if ( constant.val == "True" )
      {
        value = std::make_shared<ValueMessage<bool>>( true );
      }
      else if ( constant.val == "False" )
      {
        value = std::make_shared<ValueMessage<bool>>( false );
      }
      else
      {
        value = std::make_shared<ValueMessage<bool>>( static_cast<bool>(std::stoi( constant.val )));
      }
    }
    else if ( constant.type == "int8" || constant.type == "byte" )
    {
      value = std::make_shared<ValueMessage<int8_t>>( static_cast<int8_t>(std::stoi( constant.val )));
    }
    else if ( constant.type == "uint16" )
    {
      value = std::make_shared<ValueMessage<uint16_t>>( static_cast<uint16_t>(std::stoi( constant.val )));
    }
    else if ( constant.type == "int16" )
    {
      value = std::make_shared<ValueMessage<int16_t>>( static_cast<int16_t>(std::stoi( constant.val )));
    }
    else if ( constant.type == "uint32" )
    {
      value = std::make_shared<ValueMessage<uint32_t>>( static_cast<uint32_t>(std::stoul( constant.val )));
    }
    else if ( constant.type == "int32" )
    {
      value = std::make_shared<ValueMessage<int32_t>>( static_cast<int32_t>(std::stol( constant.val )));
    }
    else if ( constant.type == "uint64" )
    {
      value = std::make_shared<ValueMessage<uint64_t>>( static_cast<uint64_t>(std::stoul( constant.val )));
    }
    else if ( constant.type == "int64" )
    {
      value = std::make_shared<ValueMessage<int64_t>>( static_cast<int64_t>(std::stol( constant.val )));
    }
    else if ( constant.type == "float32" )
    {
      value = std::make_shared<ValueMessage<float>>( static_cast<float>(std::stof( constant.val )));
    }
    else if ( constant.type == "float64" )
    {
      value = std::make_shared<ValueMessage<double>>( static_cast<double>(std::stod( constant.val )));
    }
    else if ( constant.type == "string" )
    {
      value = std::make_shared<ValueMessage<std::string>>( constant.val );
    }
    msg_template->constants.insert( { constant.name, value } );
  }
  msg_template->compound.names = spec.names;
  for ( size_t i = 0; i < spec.types.size(); ++i )
  {
    std::string type = spec.types[i];
    std::string::size_type index_array = type.find( '[' );
    std::string type_name = type.substr( 0, index_array );
    if ( type_name == "Header" )
    {
      type_name = "std_msgs/Header";
    }
    else if ( builtin_types_.find( type_name ) == builtin_types_.end() && type_name.find( '/' ) == std::string::npos )
    {
      // Type is relative
      std::string package = spec.package;
      type_name = package.append( "/" ).append( type_name );
    }
    MessageDescription::ConstPtr sub_message_description = getMessageDescription( type_name );
    if ( sub_message_description == nullptr ) return nullptr;
    MessageTemplate::ConstPtr sub_template = sub_message_description->message_template;
    // Check if the declared type is an array by checking if the type contains an array specifier
    if ( index_array == std::string::npos )
    {
      msg_template->compound.types.push_back( sub_template );
    }
    else // It's an array
    {
      std::string::size_type index_end_array = type.find( ']', index_array );
      ssize_t length = index_end_array - index_array <= 1
                       ? -1
                       : std::stol( type.substr( index_array + 1, index_end_array - index_array - 1 ));
      MessageTemplate::Ptr array_template = std::make_shared<MessageTemplate>();
      array_template->type = MessageTypes::Array;
      array_template->array.length = length;
      array_template->array.element_type = sub_template->type;
      array_template->array.element_template = sub_template;
      msg_template->compound.types.push_back( array_template );
    }
  }
  return msg_template;
}

MessageDescription::ConstPtr DescriptionProvider::registerMessage( const DescriptionProvider::MessageSpec &spec,
                                                                   const std::string &definition )
{
  auto it = message_descriptions_.find( spec.name );
  if ( it != message_descriptions_.end()) return it->second;
  MessageDescription::Ptr description = std::make_shared<MessageDescription>();
  description->datatype = spec.name;
  description->message_definition = definition;
  description->md5 = spec.md5;
  description->specification = spec.text;

  description->message_template = createTemplate( spec );
  if ( description->message_template == nullptr ) return nullptr;

  msg_specs_.insert( { spec.name, spec } );
  message_descriptions_.insert( { spec.name, description } );
  return description;
}

MessageDescription::ConstPtr DescriptionProvider::registerMessage( const std::string &type,
                                                                   const std::string &specification )
{
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );
  if ( type == "Header" ) package = "std_msgs";
  MessageSpec spec = createSpec( type, package, specification );
  if ( spec.md5.empty())
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Failed to compute MD5 for message '%s'!", type.c_str());
    return nullptr;
  }
  return registerMessage( spec, computeFullText( spec ));
}

MessageDescription::ConstPtr DescriptionProvider::registerMessage( const std::string &type,
                                                                   const std::string &definition,
                                                                   const std::string &md5,
                                                                   const std::string &specification )
{
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );
  if ( type == "Header" ) package = "std_msgs";
  MessageSpec spec = createSpec( type, package, specification );
  if ( spec.md5.empty())
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Failed to compute MD5 for message '%s'!", type.c_str());
    return nullptr;
  }
  if ( spec.md5 != md5 )
  {
    ROS_WARN_NAMED( "RosBabelFish", "Registered MD5 for message '%s' differed from computed!", type.c_str());
    spec.md5 = md5;
  }
  return registerMessage( spec, definition );
}

ServiceDescription::ConstPtr DescriptionProvider::registerService( const std::string &type, const std::string &md5,
                                                                   const std::string &specification,
                                                                   const DescriptionProvider::MessageSpec &req_spec,
                                                                   const std::string &req_definition,
                                                                   const DescriptionProvider::MessageSpec &resp_spec,
                                                                   const std::string &resp_definition )
{
  auto it = service_descriptions_.find( type );
  if ( it != service_descriptions_.end()) return it->second;
  ServiceDescription::Ptr description = std::make_shared<ServiceDescription>();
  description->datatype = type;
  description->md5 = md5;
  description->specification = specification;

  description->request = registerMessage( req_spec, req_definition );

  description->response = registerMessage( resp_spec, resp_definition );

  service_descriptions_.insert( { type, description } );
  return description;
}

ServiceDescription::ConstPtr DescriptionProvider::registerService( const std::string &type,
                                                                   const std::string &specification,
                                                                   const std::string &req_specification,
                                                                   const std::string &resp_specification )
{
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );
  MessageSpec request_spec = createSpec( type + "Request", package, req_specification );
  MessageSpec response_spec = createSpec( type + "Response", package, resp_specification );

  MD5_CTX md5_ctx;
  MD5_Init( &md5_ctx );
  std::string md5_text = computeMD5Text( request_spec );
  MD5_Update( &md5_ctx, md5_text.data(), md5_text.length());
  md5_text = computeMD5Text( response_spec );
  MD5_Update( &md5_ctx, md5_text.data(), md5_text.length());

  unsigned char md5_digest[MD5_DIGEST_LENGTH];
  MD5_Final( md5_digest, &md5_ctx );
  return registerService( type, md5ToString( md5_digest ), specification, request_spec, computeFullText( request_spec ),
                          response_spec, computeFullText( response_spec ));
}

DescriptionProvider::MessageSpec DescriptionProvider::createSpec( const std::string &type, const std::string &package,
                                                                  const std::string &specification )
{
  MessageSpec spec;
  spec.name = type.find( '/' ) == std::string::npos ? package + '/' + type : type;
  spec.package = package;
  spec.text = specification;

  static std::regex constant_regex( R"(^\s*(\w+)\s+([a-zA-Z]\w*)\s*=\s*(.*\S)\s*$)" );
  static std::regex strip_comment_regex( R"(([^#]*[^#\s])\s*(?:#.*)?)" );
  static std::regex field_regex( R"(^\s*((\w+\/?\w+)(?:\s*\[\d*\])?)\s*(\w+)\s*)" );
  std::smatch match;
  std::string definition;
  definition.reserve( 8192 );
  std::string::size_type start = 0;
  std::string::size_type end;
  while ( true )
  {
    end = specification.find( '\n', start );
    std::string::const_iterator first = specification.begin() + start;
    std::string::const_iterator last = end == std::string::npos ? specification.end() : specification.begin() + end;
    if ( std::regex_search( first, last, match, constant_regex ) && match.size() == 4 )
    {
      std::string constant_type = match.str( 1 );
      std::string name = match.str( 2 );
      std::string val = match.str( 3 );
      if ( constant_type != "string" )
      {
        std::regex_search( val.cbegin(), val.cend(), match, strip_comment_regex );
        val = match.str( 1 );
      }
      spec.constants.push_back(
        MessageSpec::Constant{ .type = constant_type, .name = name, .val = val } );
    }
    else if ( std::regex_search( first, last, match, field_regex ) && match.size() == 4 )
    {
      std::string dep_type = match.str( 2 );
      if ( dep_type == "Header" ) dep_type = "std_msgs/Header";
      if ( !isBuiltIn( dep_type ))
      {
        if ( dep_type.find( '/' ) == std::string::npos ) dep_type.insert( 0, spec.package + '/' );
        if ( std::find( spec.dependencies.begin(), spec.dependencies.end(), dep_type ) == spec.dependencies.end())
        {
          spec.dependencies.push_back( dep_type );
        }
      }
      spec.types.push_back( match.str( 1 ));
      spec.names.push_back( match.str( 3 ));
    }

    if ( end == std::string::npos ) break;
    start = end + 1;
  }

  loadDependencies( spec );

  std::string md5_text = computeMD5Text( spec );
  unsigned char md5_digest[MD5_DIGEST_LENGTH];
  MD5( reinterpret_cast<const unsigned char *>(md5_text.data()), md5_text.length(), md5_digest );
  spec.md5 = md5ToString( md5_digest );
  return spec;
}

std::vector<std::string> DescriptionProvider::getAllDepends( const MessageSpec &spec )
{
  std::vector<std::string> result;
  for ( auto &dependency : spec.dependencies )
  {
    std::string::size_type pos_separator = dependency.find( '/' );
    std::string type = pos_separator != std::string::npos ? dependency : spec.package + '/' + dependency;
    if ( std::find( result.begin(), result.end(), type ) == result.end()) result.push_back( type );
    auto it = msg_specs_.find( type );
    std::vector<std::string> sub_deps = getAllDepends( it->second );
    for ( auto &s : sub_deps )
    {
      if ( std::find( result.begin(), result.end(), s ) != result.end()) continue;
      result.push_back( s );
    }
  }
  return result;
}

void DescriptionProvider::loadDependencies( const MessageSpec &spec )
{
  for ( auto &dependency : spec.dependencies )
  {
    std::string::size_type pos_separator = dependency.find( '/' );
    std::string type = pos_separator != std::string::npos ? dependency : spec.package + '/' + dependency;
    if ( msg_specs_.find( type ) != msg_specs_.end()) continue;
    getMessageDescription( type );
  }
}

std::string DescriptionProvider::computeFullText( const MessageSpec &spec )
{
  static std::string separator = "================================================================================\n";
  std::string result = spec.text;
  result.reserve( 8192 );
  result += '\n';
  std::vector<std::string> dependencies = getAllDepends( spec );
  for ( auto &dependency : dependencies )
  {
    result += separator;
    result += "MSG: ";
    result += dependency;
    result += '\n';
    result += msg_specs_.find( dependency )->second.text;
    result += '\n';
  }

  if ( result.length() >= 2 && result[result.length() - 2] == '\n' && result[result.length() - 1] == '\n' )
    result.pop_back();
  return result;
}

std::string DescriptionProvider::computeMD5Text( const MessageSpec &spec )
{
  std::string buffer;
  buffer.reserve( 8192 );
  for ( auto &c : spec.constants )
  {
    buffer += c.type;
    buffer += ' ';
    buffer += c.name;
    buffer += '=';
    buffer += c.val;
    buffer += '\n';
  }
  for ( size_t index = 0; index < spec.types.size(); ++index )
  {
    std::string::size_type array_pos = spec.types[index].find( '[' );
    std::string type = array_pos == std::string::npos ? spec.types[index] : spec.types[index].substr( 0, array_pos );
    if ( isBuiltIn( type ))
    {
      buffer += spec.types[index];
      buffer += ' ';
      buffer += spec.names[index];
    }
    else
    {
      std::string::size_type pos_separator = type.find( '/' );
      if ( pos_separator == std::string::npos )
        type.insert( 0, (type == "Header" ? "std_msgs" : spec.package) + '/' );
      auto it = msg_specs_.find( type );
      if ( it == msg_specs_.end()) return {};
      std::string sub_md5 = computeMD5Text( it->second );
      if ( sub_md5.empty()) return {};
      unsigned char md5_digest[MD5_DIGEST_LENGTH];
      MD5( reinterpret_cast<const unsigned char *>(sub_md5.data()), sub_md5.length(), md5_digest );
      buffer += md5ToString( md5_digest );
      buffer += ' ';
      buffer += spec.names[index];
    }
    buffer += '\n';
  }
  if ( !buffer.empty())
    buffer.pop_back(); // Remove trailing newline
  return buffer;
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

  // Deprecated
  builtin_types_.insert( "char" );
  message_descriptions_.insert( makeDescription( "char", MessageTypes::UInt8 ));

  builtin_types_.insert( "byte" );
  message_descriptions_.insert( makeDescription( "byte", MessageTypes::Int8 ));
}
} // ros_babel_fish
