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

#include "remove_last_nav_goal_tool.h"

#include <rviz/viewport_mouse_event.h>

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
RemoveLastNavGoal::RemoveLastNavGoal() {
  shortcut_key_ = 'd';
  updateTopic();
}

RemoveLastNavGoal::~RemoveLastNavGoal() {}

int RemoveLastNavGoal::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  if (event->type() == QKeyEvent::KeyPress)
  {
    int key = event->key();
    if (key == Qt::Key_Backspace || key == Qt::Key_Delete)
    {
      cancel_pt_pub_.publish(actionlib_msgs::GoalID());
    }
  }
  return 0;
}

void RemoveLastNavGoal::updateTopic()
{
  cancel_pt_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
}

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

}  // namespace rallycar_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rallycar_rviz_plugin::RemoveLastNavGoal, rviz::Tool)
// END_TUTORIAL
