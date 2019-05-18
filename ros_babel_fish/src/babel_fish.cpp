// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/babel_fish.h"

#include "ros_babel_fish/message_types/array_message.h"
#include "ros_babel_fish/message_types/compound_message.h"
#include "ros_babel_fish/message_creation.h"
#include "ros_babel_fish/message_types/value_message.h"

#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <Python.h>
#include <regex>

namespace ros_babel_fish
{

namespace
{
PyObject *importModule( const char *name )
{
  PyObject *module_string = PyString_FromString( name );
  PyObject *module = PyImport_Import( module_string );
  Py_DECREF( module_string );
  return module;
}

void checkError()
{
  PyObject *err = PyErr_Occurred();
  if ( err != nullptr )
  {
    PyObject *ptype, *pvalue, *ptraceback;
    PyObject *pystr, *module_name, *pyth_module, *pyth_func;
    char *str;

    PyErr_Fetch( &ptype, &pvalue, &ptraceback );
    pystr = PyObject_Str( pvalue );
    str = PyString_AsString( pystr );
    char *error_description = strdup( str );
    std::cout << "Error description:" << std::endl << error_description << std::endl;

    /* See if we can get a full traceback */
    module_name = PyString_FromString( "traceback" );
    pyth_module = PyImport_Import( module_name );
    Py_DECREF( module_name );

    if ( pyth_module != nullptr )
    {
      pyth_func = PyObject_GetAttrString( pyth_module, "format_exception" );
      if ( pyth_func && PyCallable_Check( pyth_func ))
      {
        PyObject *pyth_val;

        pyth_val = PyObject_CallFunctionObjArgs( pyth_func, ptype, pvalue, ptraceback, NULL );

        pystr = PyObject_Str( pyth_val );
        str = PyString_AsString( pystr );
        char *full_backtrace = strdup( str );
        Py_XDECREF( pyth_val );
        std::cout << "Full backtrace:" << std::endl << full_backtrace << std::endl;
      }
      Py_DECREF( pyth_func );
    }
  }
}

struct scoped_decref
{
  scoped_decref( PyObject *obj ) : object( obj ) { }

  ~scoped_decref()
  {
    Py_DECREF( object );
  }

  PyObject *object;
};
}

BabelFish::BabelFish()
{
  initPython();

  initBuiltInTypes();
}

BabelFish::~BabelFish()
{
  Py_DECREF( msg_context_ );
  Py_DECREF( msg_search_paths_ );
  Py_DECREF( srv_search_paths_ );

  Py_DECREF( genmsg_compute_md5_ );
  Py_DECREF( genmsg_compute_full_text_ );
  Py_DECREF( genmsg_load_depends_ );
  Py_DECREF( genmsg_load_msg_by_type_ );

  for ( ssize_t i = py_objects_.size() - 1; i >= 0; --i )
  {
    Py_DECREF( static_cast<PyObject *>(py_objects_[i]));
  }
  py_objects_.clear();
}

const MessageDescription::ConstPtr &BabelFish::getMessageDescription( const std::string &type )
{
  // Check cache
  for ( auto &entry : message_descriptions_ )
  {
    if ( entry->datatype == type ) return entry;
  }

  // If not in cache look it up
  ssize_t index = loadMessage( type );
  if ( index < 0 || static_cast<size_t>(index) >= message_descriptions_.size())
  {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return message_descriptions_[index];
}

const ServiceDescription::ConstPtr &BabelFish::getServiceDescription( const std::string &type )
{
  // Check cache
  for ( auto &entry : service_descriptions_ )
  {
    if ( entry->datatype == type ) return entry;
  }

  // If not in cache look it up
  ssize_t index = loadService( type );
  if ( index < 0 || static_cast<size_t>(index) >= service_descriptions_.size())
  {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  return service_descriptions_[index];
}

const MessageTemplate::ConstPtr &BabelFish::getMessageTemplate( const std::string &type )
{
  // Check cache
  for ( auto &entry : message_descriptions_ )
  {
    if ( entry->datatype == type ) return entry->message_template;
  }

  // If not in cache look it up
  ssize_t index = loadMessage( type );
  if ( index < 0 || static_cast<size_t>(index) >= message_descriptions_.size())
  {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return message_descriptions_[index]->message_template;
}

TranslatedMessage::Ptr BabelFish::translateMessage( const BabelFishMessage::ConstPtr &msg )
{
  ssize_t index = -1;
  const std::string &type = msg->dataType();
  const std::string &md5 = msg->md5Sum();
  for ( size_t i = 0; i < message_descriptions_.size(); ++i )
  {
    if ( message_descriptions_[i]->datatype == type )
    {
      if ( message_descriptions_[i]->md5 != md5 )
      {
        ROS_WARN_ONCE( "BabelFish knows a message of the type '%s' but with a different md5 sum!", type.c_str());
      }
      index = i;
      break;
    }
  }
  if ( index == -1 )
  {
    const std::string &message_definition = msg->definition();
    std::string::size_type end_of_message = message_definition.find( "\n==================" );
    std::string spec = message_definition.substr( 0, end_of_message );
    index = registerMessage( type, message_definition, md5, spec );
  }
  MessageTemplate::ConstPtr msg_template = message_descriptions_[index]->message_template;
  const uint8_t *stream = msg->buffer();
  size_t bytes_read = 0;

  Message::Ptr translated = createMessageFromTemplate( *msg_template, stream, msg->size(), bytes_read );
  if ( bytes_read != msg->size())
  {
    ROS_ERROR( "Translated message did not consume all message bytes! BabelFish made a mistake!" );
  }
  return std::make_shared<TranslatedMessage>( msg, translated );
}

Message::Ptr BabelFish::translateMessage( const BabelFishMessage &msg )
{
  ssize_t index = -1;
  const std::string &type = msg.dataType();
  const std::string &md5 = msg.md5Sum();
  for ( size_t i = 0; i < message_descriptions_.size(); ++i )
  {
    if ( message_descriptions_[i]->datatype == type )
    {
      if ( message_descriptions_[i]->md5 != md5 )
      {
        ROS_WARN_ONCE( "BabelFish knows a message of the type '%s' but with a different md5 sum!", type.c_str());
      }
      index = i;
      break;
    }
  }
  if ( index == -1 )
  {
    const std::string &message_definition = msg.definition();
    std::string::size_type end_of_message = message_definition.find( "\n==================" );
    std::string spec = message_definition.substr( 0, end_of_message );
    index = registerMessage( type, message_definition, md5, spec );
  }
  MessageTemplate::ConstPtr msg_template = message_descriptions_[index]->message_template;
  const uint8_t *stream = msg.buffer();
  size_t bytes_read = 0;

  Message::Ptr translated = createMessageFromTemplate( *msg_template, stream, msg.size(), bytes_read );
  if ( bytes_read != msg.size())
  {
    ROS_ERROR( "Translated message did not consume all message bytes! BabelFish made a mistake!" );
  }
  return translated;
}

BabelFishMessage::Ptr BabelFish::translateMessage( const Message::ConstPtr &msg )
{
  return translateMessage( *msg );
}

BabelFishMessage::Ptr BabelFish::translateMessage( const Message &msg )
{
  auto compound_msg = dynamic_cast<const CompoundMessage *>(&msg);
  if ( compound_msg == nullptr )
    throw BabelFishException( "Tried to translate message that is not a compound message!" );

  BabelFishMessage::Ptr result( new BabelFishMessage());
  MessageDescription::ConstPtr description = getMessageDescription( compound_msg->datatype());
  result->morph( description->md5, description->datatype, description->message_definition, "0" );
  result->allocate( msg.size());
  msg.writeToStream( result->buffer());
  return result;
}

ros::Publisher BabelFish::advertise( ros::NodeHandle &nh, const std::string &type, const std::string &topic,
                                     uint32_t queue_size_, bool latch,
                                     const ros::SubscriberStatusCallback &connect_cb )
{
  MessageDescription::ConstPtr description = getMessageDescription( type );
  ros::AdvertiseOptions opts( topic, queue_size_, description->md5, description->datatype,
                              description->message_definition, connect_cb );
  opts.latch = latch;

  return nh.advertise( opts );
}

ros::ServiceServer BabelFish::advertiseService( ros::NodeHandle &nh, const std::string &type,
                                                const std::string &service,
                                                const std::function<bool( Message::Ptr &, Message::Ptr & )> &callback )
{
  ServiceDescription::ConstPtr description = getServiceDescription( type );
  ros::AdvertiseServiceOptions opts;
  opts.datatype = description->datatype;
  opts.service = service;
  opts.req_datatype = description->request->datatype;
  opts.res_datatype = description->response->datatype;
  opts.md5sum = description->md5;
  opts.helper = ros::ServiceCallbackHelperPtr( new ros::ServiceCallbackHelperT<ServiceSpec>(
    [ this, &callback ]( BabelFishMessage::Ptr &req, BabelFishMessage::Ptr &res ) -> bool
    {
      TranslatedMessage::Ptr translated_req = translateMessage( req );
      TranslatedMessage::Ptr translated_res = translateMessage( res );
      bool result = callback( translated_req->translated_message, translated_res->translated_message );
      res = translateMessage( translated_res->translated_message );
      return result;
    },
    [ description ]() -> BabelFishMessage::Ptr
    {
      BabelFishMessage::Ptr message = boost::make_shared<BabelFishMessage>();
      message->morph( description->request, description->md5 );
      return message;
    },
    [ description ]() -> BabelFishMessage::Ptr
    {
      BabelFishMessage::Ptr message = boost::make_shared<BabelFishMessage>();
      message->morph( description->response, description->md5 );
      return message;
    }
  ));
  return nh.advertiseService( opts );
}

Message::Ptr BabelFish::createMessage( const std::string &type )
{
  MessageTemplate::ConstPtr msg_template = getMessageTemplate( type );
  return createEmptyMessageFromTemplate( *msg_template );
}

Message::Ptr BabelFish::createServiceRequest( const std::string &type )
{
  const ServiceDescription::ConstPtr &service = getServiceDescription( type );
  return createEmptyMessageFromTemplate( *service->request->message_template );
}

bool BabelFish::callService( const std::string &service, const Message::ConstPtr &req, TranslatedMessage::Ptr &res )
{
  const std::string &datatype = req->as<CompoundMessage>().datatype();
  if ( strcmp( datatype.c_str() + datatype.length() - 7, "Request" ) != 0 )
  {
    throw BabelFishException( "BabelFish can't call a service with a message that is not a request!" );
  }
  const ServiceDescription::ConstPtr &description = getServiceDescription( datatype.substr( 0, datatype.length() - 7 ));
  BabelFishMessage::Ptr request = translateMessage( req );
  BabelFishMessage::Ptr response = boost::make_shared<BabelFishMessage>();
  response->morph( description->response );
  bool result = ros::service::call( service, *request, *response );
  res = translateMessage( response );
  return result;
}

MessageTemplate::Ptr BabelFish::createTemplate( const std::string &type, const std::string &specification )
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
      MessageTemplate::ConstPtr sub_template = getMessageTemplate( type_name );
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

ssize_t BabelFish::registerMessage( const std::string &type, const std::string &message_definition,
                                    const std::string &md5, const std::string &specification )
{
  MessageDescription::Ptr description = std::make_shared<MessageDescription>(); // TODO Error handling
  description->datatype = type;
  description->message_definition = message_definition;
  description->md5 = md5;
  description->specification = specification;
  description->message_template = createTemplate( type, specification );

  size_t index = message_descriptions_.size();
  message_descriptions_.push_back( description );
  return index;
}

ssize_t BabelFish::registerService( const std::string &type, const std::string &md5, const std::string &specification,
                                    const std::string &req_message_definition, const std::string &req_md5,
                                    const std::string &req_specification,
                                    const std::string &resp_message_definition, const std::string &resp_md5,
                                    const std::string &resp_specification )
{
  ServiceDescription::Ptr description = std::make_shared<ServiceDescription>();
  description->datatype = type;
  description->md5 = md5;
  description->specification = specification;

  size_t index = registerMessage( type + "Request", req_message_definition, req_md5, req_specification );
  description->request = message_descriptions_[index];

  index = registerMessage( type + "Response", resp_message_definition, resp_md5, resp_specification );
  description->response = message_descriptions_[index];

  index = service_descriptions_.size();
  service_descriptions_.push_back( description );
  return index;
}

//! ============================= HERE STARTS THE NASTY PART =============================

ssize_t BabelFish::loadMessage( const std::string &type )
{
  // Load type
  PyObject *type_py = PyString_FromString( type.c_str());
  PyObject *args = PyTuple_Pack( 3, msg_context_, type_py, msg_search_paths_ );
  PyObject *spec = PyObject_CallObject( static_cast<PyObject *>(genmsg_load_msg_by_type_), args );
  Py_DECREF( args );
  Py_DECREF( type_py );
  if ( spec == nullptr )
  {
    // Couldn't find message
    ROS_ERROR( "Failed to look up message of type '%s'!", type.c_str());
    return -1;
  }

  // Load depends
  args = PyTuple_Pack( 3, msg_context_, spec, msg_search_paths_ );
  PyObject *r = PyObject_CallObject( static_cast<PyObject *>(genmsg_load_depends_), args );
  Py_XDECREF( r );
  Py_DECREF( args );

  {
    PyObject *full_name_text = PyObject_GetAttrString( spec, "full_name" );
    args = PyTuple_Pack( 1, full_name_text );
    PyObject *get_depends = PyObject_GetAttrString( static_cast<PyObject *>(msg_context_), "get_all_depends" );
    PyObject *depends_list = PyObject_CallObject( get_depends, args );
    Py_DECREF( args );
    Py_DECREF( get_depends );
    Py_DECREF( full_name_text );
    PyObject *iterator = PyObject_GetIter( depends_list );
    PyObject *item = nullptr;
    while ((item = PyIter_Next( iterator )) != nullptr )
    {
      std::string dependency = PyString_AsString( item );
      Py_DECREF( item );
      if ( dependency != type )
      {
        bool found = false;
        for ( auto &entry : message_descriptions_ )
        {
          if ( entry->datatype != dependency ) continue;
          found = true;
          break;
        }
        if ( !found ) loadMessage( dependency );
      }
    }
    Py_DECREF( iterator );
    Py_DECREF( depends_list );
  }

  // Compute full text
  args = PyTuple_Pack( 2, msg_context_, spec );
  PyObject *full_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_full_text_), args );
  std::string message_definition = PyString_AsString( full_text );
  Py_DECREF( full_text );

  // Get MD5
  PyObject *md5_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_md5_), args );
  Py_DECREF( args );
  std::string md5 = PyString_AsString( md5_text );
  Py_DECREF( md5_text );

  // Get specification
  PyObject *spec_text = PyObject_GetAttrString( spec, "text" );
  std::string specification = PyString_AsString( spec_text );
  Py_DECREF( spec_text );

  return registerMessage( type, message_definition, md5, specification );
}

ssize_t BabelFish::loadService( const std::string &type )
{
  // Load type
  PyObject *type_py = PyString_FromString( type.c_str());
  PyObject *args = PyTuple_Pack( 3, msg_context_, type_py, srv_search_paths_ );
  PyObject *spec = PyObject_CallObject( static_cast<PyObject *>(genmsg_load_srv_by_type_), args );
  Py_DECREF( args );
  Py_DECREF( type_py );

  // Load depends
  args = PyTuple_Pack( 3, msg_context_, spec, msg_search_paths_ );
  PyObject *r = PyObject_CallObject( static_cast<PyObject *>(genmsg_load_depends_), args );
  Py_XDECREF( r );
  Py_DECREF( args );

  // Get MD5
  args = PyTuple_Pack( 2, msg_context_, spec );
  PyObject *md5_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_md5_), args );
  Py_DECREF( args );
  std::string md5 = PyString_AsString( md5_text );
  Py_DECREF( md5_text );

  // Get specification
  PyObject *spec_text = PyObject_GetAttrString( spec, "text" );
  std::string specification = PyString_AsString( spec_text );
  Py_DECREF( spec_text );

  std::string request_message_definition;
  std::string request_md5;
  std::string request_specification;

  // Load request
  {
    PyObject *request = PyObject_GetAttrString( spec, "request" );

    // Compute full text
    args = PyTuple_Pack( 2, msg_context_, request );
    PyObject *full_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_full_text_), args );
    request_message_definition = PyString_AsString( full_text );
    Py_DECREF( full_text );

    // Get MD5
    PyObject *md5_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_md5_), args );
    Py_DECREF( args );
    request_md5 = PyString_AsString( md5_text );
    Py_DECREF( md5_text );

    // Get specification
    PyObject *spec_text = PyObject_GetAttrString( request, "text" );
    request_specification = PyString_AsString( spec_text );
    Py_DECREF( spec_text );

    Py_DECREF( request );
  }

  std::string response_message_definition;
  std::string response_md5;
  std::string response_specification;

  // Load response
  {
    PyObject *response = PyObject_GetAttrString( spec, "response" );

    // Compute full text
    args = PyTuple_Pack( 2, msg_context_, response );
    PyObject *full_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_full_text_), args );
    response_message_definition = PyString_AsString( full_text );
    Py_DECREF( full_text );

    // Get MD5
    PyObject *md5_text = PyObject_CallObject( static_cast<PyObject *>(genmsg_compute_md5_), args );
    Py_DECREF( args );
    response_md5 = PyString_AsString( md5_text );
    Py_DECREF( md5_text );

    // Get specification
    PyObject *spec_text = PyObject_GetAttrString( response, "text" );
    response_specification = PyString_AsString( spec_text );
    Py_DECREF( spec_text );

    Py_DECREF( response );
  }

  return registerService( type, md5, specification,
                          request_message_definition, request_md5, request_specification,
                          response_message_definition, response_md5, response_specification );
}

void BabelFish::initPython()
{
  Py_Initialize();
  // Import modules
  PyObject *catkin_find_in_workspaces = importModule( "catkin.find_in_workspaces" ); // TODO Error checking
  py_objects_.push_back( catkin_find_in_workspaces );

  PyObject *rospkg = importModule( "rospkg" );
  py_objects_.push_back( rospkg );

  PyObject *genmsg = importModule( "genmsg" );
  py_objects_.push_back( genmsg );

  PyObject *gentools = importModule( "genmsg.gentools" );
  py_objects_.push_back( gentools );

  PyObject *msg_loader = importModule( "genmsg.msg_loader" );
  py_objects_.push_back( msg_loader );

  PyObject *os_path = importModule( "os.path" );
  py_objects_.push_back( os_path );

  genmsg_compute_full_text_ = PyObject_GetAttrString( genmsg, "compute_full_text" );
  genmsg_compute_md5_ = PyObject_GetAttrString( genmsg, "compute_md5" );
  genmsg_load_depends_ = PyObject_GetAttrString( genmsg, "load_depends" );
  genmsg_load_msg_by_type_ = PyObject_GetAttrString( genmsg, "load_msg_by_type" );
  genmsg_load_srv_by_type_ = PyObject_GetAttrString( genmsg, "load_srv_by_type" );
  {
    PyObject *genmsg_context = PyObject_GetAttrString( genmsg, "MsgContext" );
    PyObject *genmsg_context_create = PyObject_GetAttrString( genmsg_context, "create_default" );
    msg_context_ = PyObject_CallObject( genmsg_context_create, nullptr );
    Py_DECREF( genmsg_context );
    Py_DECREF( genmsg_context_create );
  }

  PyObject *find_in_workspaces = PyObject_GetAttrString( catkin_find_in_workspaces, "find_in_workspaces" );
  py_objects_.push_back( find_in_workspaces );

  PyObject *os_path_join = PyObject_GetAttrString( os_path, "join" );
  py_objects_.push_back( os_path_join );

  // Get search paths
  msg_search_paths_ = PyDict_New();
  srv_search_paths_ = PyDict_New();
  PyObject *rospack_class = PyObject_GetAttrString( rospkg, "RosPack" );
  py_objects_.push_back( rospack_class );
  PyObject *args = PyTuple_Pack( 0 );
  PyObject *rospack_instance = PyObject_CallObject( rospack_class, args );
  Py_DECREF( args );
  py_objects_.push_back( rospack_instance );

  PyObject *rospack_get_path = PyObject_GetAttrString( rospack_instance, "get_path" );
  py_objects_.push_back( rospack_get_path );
  char list[] = "list";
  PyObject *rospack_list = PyObject_CallMethod( rospack_instance, list, nullptr );
  py_objects_.push_back( rospack_list );
  PyObject *iterator = PyObject_GetIter( rospack_list );
  PyObject *item = nullptr;

  PyObject *msg_string = PyString_FromString( "msg" );
  PyObject *srv_string = PyString_FromString( "srv" );
  // TODO Check iter null
  while ((item = PyIter_Next( iterator )) != nullptr )
  {
    // Collect package paths
    PyObject *msg_paths = PyList_New( 0 );
    PyObject *srv_paths = PyList_New( 0 );

    PyObject *args = PyTuple_Pack( 1, item );
    PyObject *path = PyObject_CallObject( rospack_get_path, args );
    Py_DECREF( args );

    args = PyTuple_Pack( 2, path, msg_string );
    PyObject *msg_path = PyObject_CallObject( os_path_join, args );
    Py_DECREF( args );

    args = PyTuple_Pack( 2, path, srv_string );
    PyObject *srv_path = PyObject_CallObject( os_path_join, args );
    Py_DECREF( args );
    Py_DECREF( path );

    PyList_Append( msg_paths, msg_path );
    PyList_Append( srv_paths, srv_path );
    // TODO find_in_workspaces as in rosmsg/_get_package_paths

    PyDict_SetItem( static_cast<PyObject *>(msg_search_paths_), item, msg_paths );
    PyDict_SetItem( static_cast<PyObject *>(srv_search_paths_), item, srv_paths );
    Py_DECREF( path );
    Py_DECREF( item );
    Py_DECREF( msg_paths );
    Py_DECREF( srv_paths );
  }
  Py_DECREF( msg_string );
  Py_DECREF( srv_string );
  Py_DECREF( iterator );
}

namespace
{
MessageDescription::Ptr makeDescription( const std::string &name, MessageType type )
{
  MessageTemplate::Ptr msg_template = std::make_shared<MessageTemplate>();
  msg_template->type = type;
  msg_template->compound.datatype = name;

  MessageDescription::Ptr description = std::make_shared<MessageDescription>();
  description->datatype = name;
  description->message_template = msg_template;
  return description;
}
}

void BabelFish::initBuiltInTypes()
{
  builtin_types_.insert( "bool" );
  message_descriptions_.push_back( makeDescription( "bool", MessageTypes::Bool ));

  builtin_types_.insert( "uint8" );
  message_descriptions_.push_back( makeDescription( "uint8", MessageTypes::UInt8 ));

  builtin_types_.insert( "int8" );
  message_descriptions_.push_back( makeDescription( "int8", MessageTypes::Int8 ));

  builtin_types_.insert( "uint16" );
  message_descriptions_.push_back( makeDescription( "uint16", MessageTypes::UInt16 ));

  builtin_types_.insert( "int16" );
  message_descriptions_.push_back( makeDescription( "int16", MessageTypes::Int16 ));

  builtin_types_.insert( "uint32" );
  message_descriptions_.push_back( makeDescription( "uint32", MessageTypes::UInt32 ));

  builtin_types_.insert( "int32" );
  message_descriptions_.push_back( makeDescription( "int32", MessageTypes::Int32 ));

  builtin_types_.insert( "uint64" );
  message_descriptions_.push_back( makeDescription( "uint64", MessageTypes::UInt64 ));

  builtin_types_.insert( "int64" );
  message_descriptions_.push_back( makeDescription( "int64", MessageTypes::Int64 ));

  builtin_types_.insert( "float32" );
  message_descriptions_.push_back( makeDescription( "float32", MessageTypes::Float32 ));

  builtin_types_.insert( "float64" );
  message_descriptions_.push_back( makeDescription( "float64", MessageTypes::Float64 ));

  builtin_types_.insert( "string" );
  message_descriptions_.push_back( makeDescription( "string", MessageTypes::String ));

  builtin_types_.insert( "time" );
  message_descriptions_.push_back( makeDescription( "time", MessageTypes::Time ));

  builtin_types_.insert( "duration" );
  message_descriptions_.push_back( makeDescription( "duration", MessageTypes::Duration ));
}
}
