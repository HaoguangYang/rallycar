/**
 * @file rviz2_ros2launch.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief A RViz2 plugin that when clicked, opens a file dialog box for user to
 * select a ROS2 launch file (*.launch.py). It then publishes the full file
 * path as a std_msgs/String ROS2 message, into the start topic. It inherits
 * the InteractionTool tool, which means users can control the camera view
 * inside RViz when this tool is active. When the user exits the tool, it
 * publishes the full path of the previously selected launch file into the
 * cancel topic.
 *
 * @version 0.1
 * @date 2024-04-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _RVIZ2_ROS2LAUNCH_HPP_
#define _RVIZ2_ROS2LAUNCH_HPP_

#include <QFileDialog>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_default_plugins/tools/interaction/interaction_tool.hpp>
#include <std_msgs/msg/string.hpp>

namespace rallycar_rviz_plugin {

class StartLaunchFile : public rviz_default_plugins::tools::InteractionTool {
  Q_OBJECT
 public:
  StartLaunchFile();
  virtual ~StartLaunchFile() = default;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

 private:
  std::string preset_launch_file_;
  std_msgs::msg::String roslaunch_msg_;
  bool started_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr roslaunch_pub_, roslaunch_cancel_pub_;
};

}  // end namespace rallycar_rviz_plugin

#endif  // _RVIZ2_ROS2LAUNCH_HPP_
