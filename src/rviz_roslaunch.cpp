/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_roslaunch.h"

namespace rallycar_rviz_plugin {

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
StartLaunchFile::StartLaunchFile() :
  rviz::InteractionTool(),
  started_(false)
{
  shortcut_key_ = 'l';
}

void StartLaunchFile::activate() {
  roslaunch_pub_ = nh_.advertise<std_msgs::String>("/rviz_run_launch_file", 1);
  roslaunch_cancel_pub_ = nh_.advertise<std_msgs::String>("/rviz_shutdown_launch_file", 1);
  std::string fileName;
  nh_.param<std::string>("~launch_file_preset", fileName, "");
  if (!fileName.size()) {
    // if has no preset, ask with a file open dialog box
    fileName = QFileDialog::getOpenFileName(nullptr,
      QObject::tr("Open Launch File"), QDir::currentPath(), QObject::tr("ROS Launch Files (*.launch)")).toStdString();
    if (!fileName.size()) {
      // if still invalid (cancled), return doing nothing.
      close();
      return;
    }
  }
  roslaunch_msg_.data = fileName;
  roslaunch_pub_.publish(roslaunch_msg_);
  rviz::InteractionTool::activate();
  started_ = true;
}

void StartLaunchFile::deactivate() {
  if (!started_) return;
  roslaunch_cancel_pub_.publish(roslaunch_msg_);
  rviz::InteractionTool::deactivate();
  roslaunch_pub_.shutdown();
  roslaunch_cancel_pub_.shutdown();
  started_ = false;
}

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

}  // namespace rallycar_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rallycar_rviz_plugin::StartLaunchFile, rviz::Tool)
// END_TUTORIAL