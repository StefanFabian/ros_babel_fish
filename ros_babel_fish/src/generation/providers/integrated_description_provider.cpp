// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/generation/providers/integrated_description_provider.h"

#include <ros/package.h>

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

  return registerMessage( type, specification );
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

  return registerService( type,  specification, request, response );
}
} // ros_babel_fish
