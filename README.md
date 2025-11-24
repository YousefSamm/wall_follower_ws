# Wall Follower ROS 2 Workspace

Autonomous wall-following stack built in ROS 2 Humble. The code was created as part of the preparation for the ROS2 Basics certificate from The Construct and showcased live on YouTube.

## Motivation

- Earn the ROS2 Basics certificate offered by [The Construct](https://app.theconstruct.ai/).
- Demonstrate the project live on YouTube so other learners can follow along: [Recording link](https://youtu.be/58YW99AQs9Q).

## Package Layout

| Package | Purpose |
| --- | --- |
| `wall_follower` | Runtime nodes, launch files, and tests for finding and following walls. |
| `custom_interfaces` | Service and action definitions shared by the nodes. |

## Core Nodes

- `wall_follower.wall_finder.WallFind`
  - Subscribes to `/scan` to measure front and right ranges.
  - Offers the `find_wall` service (`FindWall.srv`) that rotates and advances the robot until a side wall is located, while publishing status on `/wallfound`.
- `wall_follower.wall_following.WallFollower`
  - Calls the `find_wall` service to align with a wall.
  - Tracks `/scan` data to stay 30–35 cm from the wall while keeping the front path clear.
  - Starts an `OdomRecord` action goal once the wall is detected so odometry is logged during the run.
- `wall_follower.odom_recorder.OdomServer`
  - Action server that listens to `/odom` and accumulates robot poses.
  - Publishes feedback with the traveled distance and returns the full trace (`geometry_msgs/Point[]`) when a lap is finished.

All three nodes rely on `MultiThreadedExecutor` plus explicit callback groups to keep service, action, and subscription callbacks responsive while long-running control loops execute.

## Custom Interfaces

- `FindWall.srv`
  - **Request:** empty.
  - **Response:** `bool wallfound` flag to confirm alignment.
- `OdomRecord.action`
  - **Goal:** empty; simply start logging.
  - **Result:** `geometry_msgs/Point[] list_of_odoms` storing the path.
  - **Feedback:** `float32 current_total` reporting cumulative distance.

These interfaces live under `src/custom_interfaces` and are built with `rosidl_default_generators` so they can be reused by future autonomy experiments.

## Launch Files

- `main.launch.py` – starts the wall finder, follower, and odom recorder nodes together.
- `start_wall_following.launch.py` – minimal launch that assumes services/actions are available elsewhere.

Launch files expect a simulated or real robot that exposes:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/odom` (`nav_msgs/msg/Odometry`)

## Build & Run

```bash
cd /home/yousef/ros2_workspaces/wall_follower_ws
colcon build
source install/setup.bash
ros2 launch wall_follower main.launch.py
```

For simulation, bring up your preferred Gazebo world or TurtleBot setup before launching the stack so `/scan`, `/cmd_vel`, and `/odom` topics are available.

## Testing

`wall_follower/test` contains standard linters (flake8 and pep257). Run them with:

```bash
colcon test --packages-select wall_follower
colcon test-result --verbose
```

## License

This project is released under the MIT License. See the `LICENSE` file for the full text.

