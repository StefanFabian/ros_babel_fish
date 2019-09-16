// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_EMBEDDED_PYTHON_DESCRIPTION_PROVIDER_H
#define ROS_BABEL_FISH_EMBEDDED_PYTHON_DESCRIPTION_PROVIDER_H

#include "ros_babel_fish/generation/description_provider.h"

#include <set>

namespace ros_babel_fish
{

/*!
 * Uses python C++ methods to obtain the message descriptions using the associated ros packages.
 * Might clash with other libraries if they use Python.
 */
class EmbeddedPythonDescriptionProvider : public DescriptionProvider
{
  /*!
   * Makes sure Py_Initialize but especially Py_Finalize is only called once at the end of the program execution.
   */
  struct PythonContext
  {
  public:
    PythonContext();

    ~PythonContext();

    void initialize();

  private:
    bool initialized_ = false;
  };

  static PythonContext context_;

public:
  EmbeddedPythonDescriptionProvider();

  ~EmbeddedPythonDescriptionProvider();

  MessageDescription::ConstPtr getMessageDescriptionImpl( const std::string &type ) override;

  ServiceDescription::ConstPtr getServiceDescriptionImpl( const std::string &type ) override;

private:

  void initPython();

  void *msg_context_ = nullptr;
  void *msg_search_paths_ = nullptr;
  void *srv_search_paths_ = nullptr;

  // Functions
  void *genmsg_compute_full_text_ = nullptr;
  void *genmsg_compute_md5_ = nullptr;
  void *genmsg_load_depends_ = nullptr;
  void *genmsg_load_msg_by_type_ = nullptr;
  void *genmsg_load_srv_by_type_ = nullptr;

  std::vector<void *> py_objects_;
};
} // ros_babel_fish

#endif // ROS_BABEL_FISH_EMBEDDED_PYTHON_DESCRIPTION_PROVIDER_H
