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

namespace
{
#ifdef _WIN32
const std::string OS_PATHSEP(";");
#else
const std::string OS_PATHSEP( ":" );
#endif

const std::string CATKIN_MARKER_FILE = ".catkin";

std::vector<fs::path> findWorkspaceShares()
{
  char *cmake_prefix_path = std::getenv( "CMAKE_PREFIX_PATH" );
  if ( cmake_prefix_path == nullptr ) return {};
  size_t length = std::strlen( cmake_prefix_path );
  std::vector<fs::path> paths;
  std::string path;
  size_t path_sep_index = 0;
  for ( size_t i = 0; i <= length; ++i )
  {
    if ( i == length || (cmake_prefix_path[i] == OS_PATHSEP[path_sep_index] && ++path_sep_index == OS_PATHSEP.length()))
    {
      if ( !path.empty())
      {
        fs::path catkin_marker_path = fs::path( path ) / CATKIN_MARKER_FILE;
        fs::path share_path = fs::path( path ) / "share";
        if ( fs::is_regular_file( catkin_marker_path ) && fs::is_directory( share_path ))
        {
          paths.push_back( share_path );
        }
      }
      path_sep_index = 0;
      path.clear();
      continue;
    }
    path_sep_index = 0;
    path.push_back( cmake_prefix_path[i] );
  }
  return paths;
}
}

IntegratedDescriptionProvider::IntegratedDescriptionProvider()
{
  ros::V_string packages;
  if ( !ros::package::getAll( packages ))
    throw BabelFishException( "Failed to retrieve package paths, will not be able to look up message definitions!" );


  std::vector<fs::path> workspace_shares = findWorkspaceShares();
  for ( auto &pkg : packages )
  {
    fs::path base_path = ros::package::getPath( pkg );
    std::vector<std::string> msg_paths, srv_paths;

    // First check directories returned by ros::package::getPath
    fs::path msg_path = base_path / "msg";
    if ( fs::is_directory( msg_path ))
      msg_paths.push_back( msg_path.string());
    fs::path srv_path = base_path / "srv";
    if ( fs::is_directory( srv_path ))
      srv_paths.push_back( srv_path.string());

    // Then check first result in workspaces
    for ( const fs::path &workspace_share : workspace_shares )
    {
      fs::path project_path = workspace_share / pkg;
      if ( fs::is_directory( project_path ))
      {
        if ( project_path == base_path ) break;
        fs::path workspace_msg_path = project_path / "msg";
        if ( fs::is_directory( workspace_msg_path ))
          msg_paths.push_back( workspace_msg_path.string());
        fs::path workspace_srv_path = project_path / "srv";
        if ( fs::is_directory( workspace_srv_path ))
          srv_paths.push_back( workspace_srv_path.string());
        // Only add the first match and only if it differed from the base_path
        break;
      }
    }
    if ( !msg_paths.empty()) msg_paths_.insert( { pkg, msg_paths } );
    if ( !srv_paths.empty()) srv_paths_.insert( { pkg, srv_paths } );
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
    ROS_WARN_NAMED( "RosBabelFish", "Could not find package '%s' in msg paths!", package.c_str());
    return nullptr;
  }
  const std::vector<std::string> &package_paths = it->second;
  fs::path message_path;
  for ( const auto &package_path : package_paths )
  {
    fs::path p = fs::path( package_path ) / (msg_type + ".msg");
    if ( !fs::is_regular_file( p )) continue;
    message_path = p;
    break;
  }
  if ( message_path.empty())
  {
    ROS_WARN_NAMED( "RosBabelFish", "Could not find message of type '%s' in package '%s'!", msg_type.c_str(),
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
    ROS_WARN_NAMED( "RosBabelFish", "Could not find package '%s' in srv paths!", package.c_str());
    return nullptr;
  }
  const std::vector<std::string> &package_paths = it->second;
  fs::path service_path;
  for ( const auto &package_path : package_paths )
  {
    fs::path p = fs::path( package_path ) / (msg_type + ".srv");
    if ( !fs::is_regular_file( p )) continue;
    service_path = p;
    break;
  }
  if ( service_path.empty())
  {
    ROS_WARN_NAMED( "RosBabelFish", "Could not find service of type '%s' in package '%s'!", msg_type.c_str(),
                    package.c_str());
    return nullptr;
  }

  // Load service specification from file
  std::ifstream file_input( service_path );
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

  return registerService( type, specification, request, response );
}
} // ros_babel_fish
