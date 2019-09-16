// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/generation/providers/embedded_python_description_provider.h"

#include <Python.h>

namespace ros_babel_fish
{

EmbeddedPythonDescriptionProvider::PythonContext EmbeddedPythonDescriptionProvider::context_ = EmbeddedPythonDescriptionProvider::PythonContext();

EmbeddedPythonDescriptionProvider::PythonContext::PythonContext() = default;

EmbeddedPythonDescriptionProvider::PythonContext::~PythonContext()
{
  Py_Finalize();
}

void EmbeddedPythonDescriptionProvider::PythonContext::initialize()
{
  if ( initialized_ ) return;
  initialized_ = true;
  // Makes sure two instances don't kill each others context, still not my favorite approach since it risks clashing
  // with other libraries if they use embedded python
  Py_Initialize();
}

namespace
{
PyObject *importModule( const char *name )
{
  PyObject *module_string = PyString_FromString( name );
  PyObject *module = PyImport_Import( module_string );
  Py_DECREF( module_string );
  return module;
}

// Used for debugging
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
  explicit scoped_decref( PyObject *obj ) : object( obj ) { }

  ~scoped_decref()
  {
    Py_DECREF( object );
  }

  PyObject *object;
};
}

EmbeddedPythonDescriptionProvider::EmbeddedPythonDescriptionProvider()
{
  initPython();
}

EmbeddedPythonDescriptionProvider::~EmbeddedPythonDescriptionProvider()
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

MessageDescription::ConstPtr EmbeddedPythonDescriptionProvider::getMessageDescriptionImpl( const std::string &type )
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
    std::cerr << "Failed to look up message of type '" << type.c_str() << "'!";
    return nullptr;
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
    scoped_decref depends_list_decref( depends_list );
    Py_DECREF( args );
    Py_DECREF( get_depends );
    Py_DECREF( full_name_text );
    PyObject *iterator = PyObject_GetIter( depends_list );
    scoped_decref iterator_decref( iterator );
    PyObject *item = nullptr;
    while ((item = PyIter_Next( iterator )) != nullptr )
    {
      std::string dependency = PyString_AsString( item );
      Py_DECREF( item );
      if ( dependency != type )
      {
        if ( getMessageDescription( dependency ) == nullptr )
        {
          return nullptr;
        }
      }
    }
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

  MessageDescription::Ptr description = std::make_shared<MessageDescription>();
  description->datatype = type;
  if ( message_definition.empty() || message_definition[message_definition.length() - 1] != '\n' )
    message_definition += '\n';
  description->message_definition = message_definition;
  description->md5 = md5;
  description->specification = specification;
  description->message_template = createTemplate( type, specification );
  if ( description->message_template == nullptr ) return nullptr;

  return description;
}

ServiceDescription::ConstPtr EmbeddedPythonDescriptionProvider::getServiceDescriptionImpl( const std::string &type )
{
  // Load type
  PyObject *type_py = PyString_FromString( type.c_str());
  PyObject *args = PyTuple_Pack( 3, msg_context_, type_py, srv_search_paths_ );
  PyObject *spec = PyObject_CallObject( static_cast<PyObject *>(genmsg_load_srv_by_type_), args );
  Py_DECREF( args );
  Py_DECREF( type_py );
  if ( spec == nullptr )
  {
    // Couldn't find service
    std::cerr << "Failed to look up service of type '" << type.c_str() << "'!";
    return nullptr;
  }

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

  if ( request_message_definition.empty() || request_message_definition[request_message_definition.length() - 1] != '\n' )
    request_message_definition += '\n';
  if ( response_message_definition.empty() || response_message_definition[response_message_definition.length() - 1] != '\n' )
    response_message_definition += '\n';

  return registerService( type, md5, specification,
                          request_message_definition, request_md5, request_specification,
                          response_message_definition, response_md5, response_specification );
}

void EmbeddedPythonDescriptionProvider::initPython()
{
  context_.initialize();

  // Import modules
  PyObject *catkin_find_in_workspaces = importModule( "catkin.find_in_workspaces" ); // TODO Error checking
  checkError();
  if ( catkin_find_in_workspaces == nullptr ) throw BabelFishException( "Failed to initialize description provider!" );
  py_objects_.push_back( catkin_find_in_workspaces );

  PyObject *rospkg = importModule( "rospkg" );
  checkError();
  if ( rospkg == nullptr ) throw BabelFishException( "Failed to initialize description provider!" );
  py_objects_.push_back( rospkg );

  PyObject *genmsg = importModule( "genmsg" );
  if ( rospkg == nullptr ) throw BabelFishException( "Failed to initialize description provider!" );
  py_objects_.push_back( genmsg );

  PyObject *gentools = importModule( "genmsg.gentools" );
  if ( rospkg == nullptr ) throw BabelFishException( "Failed to initialize description provider!" );
  py_objects_.push_back( gentools );

  PyObject *msg_loader = importModule( "genmsg.msg_loader" );
  if ( rospkg == nullptr ) throw BabelFishException( "Failed to initialize description provider!" );
  py_objects_.push_back( msg_loader );

  PyObject *os_path = importModule( "os.path" );
  if ( rospkg == nullptr ) throw BabelFishException( "Failed to initialize description provider!" );
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

  char list[] = "list";
  PyObject *rospack_list = PyObject_CallMethod( rospack_instance, list, nullptr );
  py_objects_.push_back( rospack_list );
  PyObject *iterator = PyObject_GetIter( rospack_list );
  PyObject *item = nullptr;

  PyObject *rospack_get_path = PyObject_GetAttrString( rospack_instance, "get_path" );
//  py_objects_.push_back( rospack_get_path ); // SegFault if this is decref in destructor
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
} // ros_babel_fish
