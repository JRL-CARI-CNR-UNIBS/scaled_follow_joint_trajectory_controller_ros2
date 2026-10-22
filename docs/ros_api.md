# ROS API

Reference of the ROS interface of `scaled_fjt_controller/ScaledFjtController`.

In the names below, `<controller>` is the name given to the controller in the controller manager
configuration (e.g. `manipulator_controller`). Relative topic names are resolved in the namespace of
the controller manager.

## Parameters

### Parameters of this controller

| Name | Type | Default | Description |
|---|---|---|---|
| `speed_ovr_topics` | string array | `["/speed_ovr", "/safe_ovr"]` | Topics (`std_msgs/msg/Int16`) carrying a speed override in percent. The global override is the product of the last value received on each topic. |
| `spline_order` | int | `1` | Order of the interpolating polynomial between waypoints: `1` continuous velocity (cubic), `2` continuous acceleration (quintic), `3` continuous jerk (7th degree, zero jerk at waypoints), `4` continuous snap (9th degree, zero jerk and snap at waypoints). Values below 1 are set to 1. |

Both parameters are read in `on_configure()`: to change them, reconfigure the controller.

Orders `≥ 1` need the `velocities` of every trajectory point, orders `≥ 2` also the
`accelerations` (MoveIt provides both).

### Inherited parameters

The controller inherits the parameters of
[`joint_trajectory_controller`](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
The relevant ones are:

| Name | Description |
|---|---|
| `joints` | Controlled joints. Goals must contain exactly these joints (any order). |
| `command_interfaces` | Any combination of `position`, `velocity`, `acceleration`: each available interface is written at every cycle. |
| `state_interfaces` | `position` is required (used to start from the current position on activation); `velocity` is optional. |
| `action_monitor_rate` | Rate (Hz) of the non-real-time timer that sends the action result. The result arrives at most `1 / action_monitor_rate` after the end of the trajectory. |

Parameters about tolerances and `goal_time` (`constraints.*`) are accepted but not used: this
controller does not check the tracking error.

## Subscribed topics

| Topic | Type | Description |
|---|---|---|
| each of `speed_ovr_topics` | `std_msgs/msg/Int16` | Speed override in percent, clamped to [0, 100]. Every topic starts at 100. |

## Published topics

All topics are published from the control loop through `realtime_tools::RealtimePublisher`: if the
publisher is still busy with the previous message, the sample is skipped.

| Topic | Type | Description |
|---|---|---|
| `scaled_time` | `std_msgs/msg/Float64` | Scaled time τ along the current trajectory, in seconds. |
| `execution_ratio` | `std_msgs/msg/Float64` | `τ / trajectory duration`, clamped to [0, 1] (1 when no trajectory is running). |
| `unscaled_joint_target` | `sensor_msgs/msg/JointState` | Trajectory point at τ with nominal (unscaled) velocities. |
| `<controller>/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | Published by the base class. Desired and actual state are both the commanded point. |

## Action server

| Action | Type |
|---|---|
| `<controller>/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |

Behaviour:

| Situation | What happens |
|---|---|
| Goal received | Validated by the base class (joint names, monotonic times, …). |
| Goal accepted | The trajectory is reordered to the order of `joints` and starts immediately, with τ = 0. The previous goal, if any, is preempted. |
| Goal with different joint names | Aborted with `INVALID_JOINTS`. |
| τ before the first waypoint | The first waypoint is held (an error is logged, at most once per second). Trajectories should start at `time_from_start = 0` from the current position. |
| End of trajectory | The last waypoint is held with zero velocity; the goal succeeds with `SUCCESSFUL` within `1 / action_monitor_rate`. |
| Override = 0 | The robot stops on the path; τ does not advance and the goal stays active. |
| Goal cancelled | The robot holds the last commanded position; the goal is canceled. |
| Controller deactivated | The active goal is aborted by the base class. On the next activation the controller holds the measured position. |
| Feedback | Not published. |

## Threads

| Code | Thread | Notes |
|---|---|---|
| `update()`, `on_activate()`, `on_deactivate()` | Control loop (real-time) | Never blocks on work proportional to the trajectory length, never logs or publishes directly. |
| Goal callbacks, override callbacks, result timer | Controller manager executor | Goal callbacks lock the control loop only to swap the interpolator. |
| `AsyncLogger` worker | Dedicated thread | Writes the log messages queued by the other threads. |
| Real-time publishers | One thread per publisher | Created by `realtime_tools`. |
