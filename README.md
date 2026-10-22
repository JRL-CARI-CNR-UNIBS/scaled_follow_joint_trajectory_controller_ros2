# scaled_follow_joint_trajectory_controller_ros2

A [ros2_control](https://control.ros.org) controller that executes
`control_msgs/action/FollowJointTrajectory` goals and lets you **scale the execution speed at
runtime**, from 0 % (robot stopped on the path) to 100 % (nominal speed), by publishing on one or
more *speed override* topics.

It is the ROS 2 port of `scaled_fjt_controller` from
[scaled_follow_joint_trajectory_controller](https://github.com/JRL-CARI-CNR-UNIBS/scaled_follow_joint_trajectory_controller)
and inherits from
[`joint_trajectory_controller/JointTrajectoryController`](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html),
so it is configured like the standard joint trajectory controller and works with MoveIt out of the box.

- Plugin name: `scaled_fjt_controller/ScaledFjtController`
- Supported distributions: ROS 2 **Humble** and **Jazzy**
- License: [Apache 2.0](LICENSE)

## How it works

The controller follows the trajectory along a *scaled time* τ instead of the wall-clock time:

```
τ(k+1) = τ(k) + Δt · s(k)
```

where `Δt` is the control period and `s ∈ [0, 1]` is the product of the last value received on each
override topic. The commanded point is the trajectory interpolated at τ, with velocities multiplied
by `s` and accelerations by `s²`. With `s = 0.5` the robot runs along the same path at half speed;
with `s = 0` it stops on the path and resumes from the same point when `s` grows again.

Between waypoints the trajectory is interpolated with polynomials whose order is set by
`spline_order` (1 = continuous velocity, up to 4 = continuous snap).

## Real-time design

The control loop (`update()`) runs at the controller manager rate (e.g. 500 Hz on a UR robot) and
must never stall: a robot like the UR drops the connection after a few tens of milliseconds without
commands. For this reason:

- **Goals are prepared outside the control loop.** Reordering and loading a new trajectory is done in
  the action callback; the control loop is locked only for a pointer swap (a few microseconds),
  whatever the trajectory length.
- **No blocking logging in the loop.** Messages go through a non-blocking queue served by a
  dedicated thread (`AsyncLogger`); interpolation errors are throttled to one per second.
- **No blocking publishing in the loop.** Topics are published with
  `realtime_tools::RealtimePublisher` (a sample is skipped if the publisher is busy).
- **The action result is sent by a non-real-time timer.** The loop only flags the end of the
  trajectory; a timer running at `action_monitor_rate` sends the result.
- **ROS entities are created in `on_configure()`**, not in `on_activate()`, which the controller
  manager runs inside the control loop.
- **Interpolation is O(1) per cycle**: the segment search starts from the segment used in the
  previous cycle.

With a mock UR10e at 500 Hz, accepting a 100 000-point goal keeps the loop period at about 2 ms,
while the previous implementation stalled the loop for ~320 ms with 20 000 points.

## Installation

```console
cd <your_ros2_ws>/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/scaled_follow_joint_trajectory_controller_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select scaled_follow_joint_trajectory_controller_ros2
source install/setup.bash
```

## Quick start

In the ros2_control configuration file:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    manipulator_controller:
      type: scaled_fjt_controller/ScaledFjtController

manipulator_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    action_monitor_rate: 20.0
    spline_order: 1                            # optional, default 1
    speed_ovr_topics: [/speed_ovr, /safe_ovr]  # optional, this is the default
```

Load and activate it, then scale the speed while a trajectory is running:

```console
ros2 topic pub --once /speed_ovr std_msgs/msg/Int16 "{data: 30}"   # 30 % of the nominal speed
```

With MoveIt, disable the execution duration monitoring, otherwise MoveIt cancels a trajectory that
was slowed down (see [moveit2#1848](https://github.com/moveit/moveit2/issues/1848)):

```yaml
# moveit_controllers.yaml
trajectory_execution:
  execution_duration_monitoring: false
```

## Documentation

| Document | Content |
|---|---|
| [Tutorial](docs/tutorial.md) | Step-by-step setup: configuration, MoveIt, speed scaling, monitoring, debugging |
| [ROS API](docs/ros_api.md) | Parameters, topics, action server and behaviour in every situation |
| C++ API | Generated with Doxygen: run `doxygen Doxyfile` in the package folder and open `docs/api/html/index.html` |

## Limitations

- The controller is open loop: the goal tolerances and `goal_time` of the joint trajectory controller
  are not checked, and the published controller state reports the commanded point as both desired
  and actual state.
- No action feedback is published.
- A trajectory should start at `time_from_start = 0` from the current robot position: before its
  first waypoint the controller holds that waypoint.

## License

Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors.
Licensed under the [Apache License, Version 2.0](LICENSE).
