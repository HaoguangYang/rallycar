/**
 * @file remove_last_nav_goal_tool.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief A RViz2 plugin that sends out a cancel-goal message when pressed.
 * @version 0.1
 * @date 2024-04-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _REMOVE_LAST_NAV_GOAL_TOOL_HPP_
#define _REMOVE_LAST_NAV_GOAL_TOOL_HPP_

#include <QKeyEvent>
#include <QObject>
#include <actionlib_msgs/msg/goal_id.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool.hpp>

namespace rallycar_rviz_plugin {

class RemoveLastNavGoal : public rviz_common::Tool {
  Q_OBJECT
 public:
  RemoveLastNavGoal();
  virtual ~RemoveLastNavGoal() = default;

  void onInitialize() override;

  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

  void activate() override{};

  void deactivate() override{};

 private:
  rclcpp::Publisher<actionlib_msgs::msg::GoalID>::SharedPtr cancel_pt_pub_;
};

}  // end namespace rallycar_rviz_plugin

#endif  // _REMOVE_LAST_NAV_GOAL_TOOL_HPP_
