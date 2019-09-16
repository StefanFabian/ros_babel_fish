// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/generation/providers/integrated_description_provider.h"

#include <ros/package.h>

#include <openssl/md5.h>
#include <experimental/filesystem>
#include <fstream>
#include <regex>

namespace fs = std::experimental::filesystem;

namespace ros_babel_fish
{

IntegratedDescriptionProvider::IntegratedDescriptionProvider()
{
  ros::V_string packages;
  if ( !ros::package::getAll( packages ))
    throw BabelFishException( "Failed to retrieve package paths, will not be able to look up message definitions!" );

  for ( auto &pkg : packages )
  {
    fs::path base_path = ros::package::getPath( pkg );
    // In theory we would also need find_in_workspaces but there seems to be no C++ version
    fs::path msg_path = base_path / "msg";
    if ( fs::is_directory( msg_path ))
      msg_paths_.insert( { pkg, msg_path.string() } );
    fs::path srv_path = base_path / "srv";
    if ( fs::is_directory( srv_path ))
      srv_paths_.insert( { pkg, srv_path.string() } );
  }
}

namespace
{
std::string BUILTIN_TYPES[] = {
  "bool",
  "uint8",
  "int8",
  "uint16",
  "int16",
  "uint32",
  "int32",
  "uint64",
  "int64",
  "float32",
  "float64",
  "string",
  "time",
  "duration",
// deprecated
  "char",
  "byte"
};

bool isBuiltIn( const std::string &type )
{
  return std::find( std::begin( BUILTIN_TYPES ), std::end( BUILTIN_TYPES ), type ) != std::end( BUILTIN_TYPES );
}

std::string md5ToString( const unsigned char md5[MD5_DIGEST_LENGTH] )
{
  std::stringstream md5string;
  md5string << std::hex << std::setfill( '0' );
  for ( unsigned int i = 0; i < MD5_DIGEST_LENGTH; ++i )
    md5string << std::setw( 2 ) << (int) md5[i];
  return md5string.str();
}
}

MessageDescription::ConstPtr IntegratedDescriptionProvider::getMessageDescriptionImpl( const std::string &type )
{
  if ( type == "Header" ) return getMessageDescription( "std_msgs/Header" );
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );
  std::string msg_type = type.substr( package.length() + 1 );
  auto it = msg_paths_.find( package );
  if ( it == msg_paths_.end())
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Could not find package '%s' in msg paths!", package.c_str());
    return nullptr;
  }
  fs::path package_path = it->second;
  std::string message_path = package_path / (msg_type + ".msg");
  if ( !fs::is_regular_file( message_path ))
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Could not find message of type '%s' in package '%s'!", msg_type.c_str(),
                     package.c_str());
    return nullptr;
  }

  // Load message specification from file
  std::ifstream file_input( message_path );
  file_input.seekg( 0, std::ios::end );
  std::string specification;
  specification.resize( file_input.tellg());
  file_input.seekg( 0, std::ios::beg );
  file_input.read( &specification[0], specification.size());
  file_input.close();

  MessageSpec spec = createSpec( type, package, specification );
  if ( spec.md5.empty())
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Failed to compute MD5 for message '%s'!", type.c_str());
    return nullptr;
  }
  msg_specs_.insert( { type, spec } );
  return registerMessage( type, computeFullText( spec ), spec.md5, specification );
}

MessageDescription::ConstPtr IntegratedDescriptionProvider::getMessageDescriptionImpl( const BabelFishMessage &msg )
{
  // This will split the message definition and register all of the embedded message types, so look ups can be avoided
  const std::string &type = msg.dataType();
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );

  const std::string &definition = msg.definition();

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
        msg_specs_.insert( { spec.name, spec } );
        registerMessage( spec.name, computeFullText( spec ), spec.md5, spec.text );
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
  msg_specs_.insert( { spec.name, spec } );
  return registerMessage( spec.name, computeFullText( spec ), spec.md5, spec.text );
}

ServiceDescription::ConstPtr IntegratedDescriptionProvider::getServiceDescriptionImpl( const std::string &type )
{
  std::string::size_type pos_separator = type.find( '/' );
  std::string package = type.substr( 0, pos_separator );
  std::string msg_type = type.substr( package.length() + 1 );
  auto it = srv_paths_.find( package );
  if ( it == srv_paths_.end())
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Could not find package '%s' in srv paths!", package.c_str());
    return nullptr;
  }
  fs::path package_path = it->second;
  std::string message_path = package_path / (msg_type + ".srv");
  if ( !fs::is_regular_file( message_path ))
  {
    ROS_DEBUG_NAMED( "RosBabelFish", "Could not find service of type '%s' in package '%s'!", msg_type.c_str(),
                     package.c_str());
    return nullptr;
  }

  // Load service specification from file
  std::ifstream file_input( message_path );
  file_input.seekg( 0, std::ios::end );
  std::string specification;
  specification.resize( file_input.tellg());
  file_input.seekg( 0, std::ios::beg );
  file_input.read( &specification[0], specification.size());
  file_input.close();

  // Why write many lines if a single not understandable regex can do the job
  // This regex is responsible for stripping comments and whitespaces from the line.
  // There are two cases:
  // First, the field is a string constant, in that case the comment character has no effect and the line is simply trimmed
  // Second, the field is not a string constant and the comment should be stripped and the rest of the line trimmed
  static std::regex line_regex(
    R"(^\s*(string\s[a-zA-Z]\w*(?:\s*=.*\S)?|\w+(?:\/\w+)?(?:\s*\[\d*\])?\s*[a-zA-Z]\w*(?:\s*=\s*[^#]*[^#\s])?))" );
  std::smatch match;
  std::string request;
  request.reserve( specification.size());
  std::string response;
  response.reserve( specification.size());
  std::string *current = &request;
  std::string::size_type start = 0;
  std::string::size_type end;
  while ( true )
  {
    end = specification.find( '\n', start );
    if ( current == &request && start < specification.length() + 2 &&
         specification[start] == '-' && specification[start + 1] == '-' && specification[start + 2] == '-' )
    {
      current = &response;
      if ( end == std::string::npos ) break;
      start = end + 1;
      continue;
    }
    std::string::const_iterator first = specification.begin() + start;
    std::string::const_iterator last = end == std::string::npos ? specification.end() : specification.begin() + end;
    if ( std::regex_search( first, last, match, line_regex ))
    {
      *current += match.str( 1 );
    }
    *current += '\n';

    if ( end == std::string::npos ) break;
    start = end + 1;
  }

  MessageSpec request_spec = createSpec( type + "Request", package, request );
  msg_specs_.insert( { request_spec.name, request_spec } );
  MessageSpec response_spec = createSpec( type + "Response", package, response );
  msg_specs_.insert( { response_spec.name, response_spec } );

  MD5_CTX md5_ctx;
  MD5_Init( &md5_ctx );
  std::string md5_text = computeMD5Text( request_spec );
  MD5_Update( &md5_ctx, md5_text.data(), md5_text.length());
  md5_text = computeMD5Text( response_spec );
  MD5_Update( &md5_ctx, md5_text.data(), md5_text.length());

  unsigned char md5_digest[MD5_DIGEST_LENGTH];
  MD5_Final( md5_digest, &md5_ctx );
  return registerService( type, md5ToString( md5_digest ), specification,
                          computeFullText( request_spec ), request_spec.md5, request_spec.text,
                          computeFullText( response_spec ), response_spec.md5, response_spec.text );
}

IntegratedDescriptionProvider::MessageSpec IntegratedDescriptionProvider::createSpec( const std::string &type,
                                                                                      const std::string &package,
                                                                                      const std::string &specification )
{
  IntegratedDescriptionProvider::MessageSpec spec;
  spec.name = type;
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

void IntegratedDescriptionProvider::loadDependencies( const MessageSpec &spec )
{
  for ( auto &dependency : spec.dependencies )
  {
    std::string::size_type pos_separator = dependency.find( '/' );
    std::string type = pos_separator != std::string::npos ? dependency : spec.package + '/' + dependency;
    if ( msg_specs_.find( type ) != msg_specs_.end()) continue;
    getMessageDescription( type );
  }
}

std::vector<std::string> IntegratedDescriptionProvider::getAllDepends( const MessageSpec &spec )
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

std::string IntegratedDescriptionProvider::computeFullText( const MessageSpec &spec )
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

std::string IntegratedDescriptionProvider::computeMD5Text( const MessageSpec &spec )
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
} // ros_babel_fish
