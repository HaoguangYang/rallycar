# MFET 642/442 Programming Robots with ROS
The RallyCar Base Software

## Included components
The package includes low-level hardware interface, as well as utilities that helps developers to build and load maps and race lines.

### Hardware interface
The hardware interface is started with the `launch/rallycar_hardware.launch` launch file. The hardware interface is realized through the standard `rosserial` package. Specifically, on the client side, we use an Arduino as the aggregator of the low-level motor drivers and an IMU sensor. The code on the Arduino is provided in the `firmware` folder as a reference. Students are not required to understand or modify the firmware. By configuring the Arduino with the `rosserial` library, the server side (onboard computer) can convert the serial communication contents to and from ROS messages directly. This is achieved using the `rosserial_python` package.
The hardware interface is subscribing to the following topics that drive the car:
- `/accelerator_cmd` :arrow_left: throttle commands with `std_msgs/Float32` message type. The effective data range is $-2048.0$ to $2048.0$. Out-of-range values will be cropped to the nearest value. A positive value means driving the car forward. $2048.0$ corresponds to full throttle. A value around $170$ ~ $210$ will make the car start to move. From a broad observation, the throttle deadzone is about $180.0$, with about $15.0$ hysteresis. No effective breaking/back-driving was observed previously despite setting the value to negative. A previous document noted about a second trigger of a negative value (negative->0->negative) will put the car in reverse. This behavior is still to be verified.
- `/steering_cmd` :arrow_left: steering commands with `std_msgs/Float32` message type. The effective data range is $-2048.0$ to $2048.0$. Out-of-range values will be cropped to the nearest value. $2048.0$ indicates full *left* steering, and $-2048.0$ indicates full right steering. The steering angle on the front wheel is about $50^{\circ}$ (or $100^{\circ}$ end-to-end). A steering command around $\pm1000.0$ gives a turning radius of about $1$ meter.
The throttle and steering commands are processed at $100$ Hz max frequency. Usually a $50$ Hz frequency is enough.

The launch file starts a static transformation publisher that gives the frames of sensor measurements and their rigid transformations:
- `/tf_static` :arrow_right: static transformations describing transformations from the base link of the car to onboard sensor reference frames. The frames are defined in `urdf/rallycar.urdf`.
The hardware interface publishes measurements of an onboard IMU sensor. Due to memory limitations of the Arduino, the IMU data is transmitted in segments (`/imu/accelerometer`, `/imu/gyroscope`, `/imu/orientation`, and `/imu/stamp`. Students, please ignore these four topics, as you should never subscribe to them directly). An `imu_parser` node is started alongside to aggregate these segments and publishes:
- `/imu` :arrow_right: aggregated IMU measurements with `sensor_msgs/Imu` message type. The topic provides full orientation, linear acceleration, and angular velocity measurements at $100$ Hz. The sensor frame is `imu_frame`. The orientation is relative to where the measurement starts. Gravitational acceleration is not removed.
The laubch file also starts the Hokuyo UST-10LX 2-D LiDAR driver. The driver publishes at:
- `/scan` :arrow_right: 2-D laser scans of the surroundings. The 1080 laser beams span 270 degrees, from right to left. The distance measurements of these beams refresh at $40$ Hz. Max range is 10 meters. The sensor frame is `laser`.
Optionally, a launch file for Intel RealSense cameras is provided at `launch/include/realsense.launch`. Students who are interested in image processing can include this file to the `rallycar_hardware.launch`.

### Path Server and Recorder
The path server and recorder (`scripts/path_server.py`) manages user interaction with waypoints in the map. The code has two working modes, that are set through its `mode` parameter.

In `record` mode, it works with the RViz visualization tool to record user-issued sequence of 2D Nav goals. It then creates or overwrites the file (specified with `file_name` parameter) upon exit, if the user's input is non-empty. The recorded file on disk is a list of `geometry_msgs/Pose` structure in `YAML` format.

In `publish` mode, it reads the file specified by the `file_name` parameter, and process it with a `YAML` parser. The loaded data is then further parsed as a `nav_msgs/Path` message type, containing *all* points within the specified file. The message is published at the `/desired_path` topic. The publisher is latched, which means every connecting node will receive this message exactly once.

### Keyboard teleoperation script
The `scripts/rally_keyboard_teleop.py` script listens on keyboard input in a terminal window at $10$ Hz, and publishes to the `/accelerator_cmd` and `/steering_cmd` topics. The values publishes to these topics depend on the last key pressed. In general, use the circle of keys `i`, `u`, `j`, `m`, `,`, `.`,`l`, `o` (case sensitive) to send constant directional speed and steering combinations relative to key `k`. If key `k` or other non-mapped keys are pressed, the command will be set to $(0,0)$. Use `w`/`x`, `e`/`c`, and `q`/`z` to control the magnitude of command components for acceleration, steering, or both. Refer to screen output for instantaneous help.

When using the key combos to adjust the command magnitude, be aware that they are changed by scaling, so it may start out small, but getting big **exponentially**.

### RViz tool plugin to help build a path with a given map
In the `src` folder we provide a plugin for RViz that works with the path server in recording mode, to help user create their own path in a natural way. In RViz, you can see a bunch of tools, such as `Interact`, `Move Camera`, etc. Towards the right, you can see a purple arrow icon with the name `2D Nav Goal`. Activae the tool with shortcut key `g` or clicking on the icon. Press `g` again, or press `esc` to exit the tool. With fixed frame set to `map` on the right panel and a map visualization layer loaded, you can then click, drag and drop your target pose on the map. The click represents the position of the goal pose, and the drag direction specifies the frontal orientation. With the path server started in `record` mode, this goal pose will be recorded sequentially, and publishes under the `/desired_path` topic as a `nav_msgs/Path` message.

In case you mistakenly picked a goal pose and want to delete it, activate the `RemoveLastNavGoal` tool by either clicking on the icon, or using the shortcut key `d`. With the tool activated, you can use key `delete` or key `backspace` to sequentially remove the last pose in the recorded sequence. The updated path will be reflected in the `/desired_path` topic as a `nav_msgs/Path` message.

This plugin is meant to be used with the path server in this package, and with the `resources/rviz_configs/build_path.rviz` RViz configuration.

## Provided launch file snippets
- `rallycar_hardware.launch`: brings up rallycar hardware -- motor driver, LiDAR scanner, and static transforms defined by the URDF of the robot.
- `build_map.launch`: builds new map using `hector_mapping` in a no-wheel-odometer setup. The user then needs to use:
    ```sh
    rosrun map_server map_saver -f ${MY_MAP_NAME}
    ```
    when finished to save the map to the disk.
- `build_path.launch`: builds new path with the provided map file and the path name. It starts the path server in `record` mode, loads the map, and starts an RViz instance for user interaction.
- `load_map.launch`: snippet that loads the specified map file using `map_server`, which publishes the map under the `/map` topic.
- `load_path.launch`: snippet that loads the specified path file using the path server under `publish` mode. It publishes the path under the `/desired_path` topic.
