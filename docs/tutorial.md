# Tutorial: speed-scaled trajectory execution

This tutorial adds `scaled_fjt_controller/ScaledFjtController` to an existing ros2_control setup,
executes trajectories from the command line and from MoveIt, changes their speed at runtime and
shows how to monitor and debug the controller.

The examples use a 6-axis arm with joints `joint_1` … `joint_6` and a controller named
`manipulator_controller`; replace them with the names of your robot.

## 1. Build

```console
cd <your_ros2_ws>/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/scaled_follow_joint_trajectory_controller_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select scaled_follow_joint_trajectory_controller_ros2
source install/setup.bash
```

Check that the plugin is visible to the controller manager:

```console
ros2 control list_controller_types | grep ScaledFjt
# scaled_fjt_controller/ScaledFjtController    controller_interface::ControllerInterface
```

## 2. Configure the controller

Add the controller to the ros2_control configuration file of your robot (the file passed to
`ros2_control_node`, or to the driver node, e.g. `ur_ros2_control_node`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    manipulator_controller:
      type: scaled_fjt_controller/ScaledFjtController

manipulator_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    action_monitor_rate: 20.0   # the action result is sent within 1/20 s from the end
    allow_partial_joints_goal: false
    spline_order: 1             # 1: cubic, continuous velocity
    speed_ovr_topics:           # global override = product of the last value on each topic
      - /speed_ovr
      - /safe_ovr
```

Choosing `spline_order`:

| Order | Polynomial | Continuous up to | Needs in each point |
|---|---|---|---|
| 1 | cubic | velocity | positions, velocities |
| 2 | quintic | acceleration | positions, velocities, accelerations |
| 3 | 7th degree | jerk (zero at waypoints) | positions, velocities, accelerations |
| 4 | 9th degree | snap (zero at waypoints) | positions, velocities, accelerations |

Trajectories planned by MoveIt contain positions, velocities and accelerations, so every order works.

## 3. Start the controller

Spawn it together with the joint state broadcaster, for example from a launch file:

```python
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "manipulator_controller",
               "--controller-manager", "/controller_manager"],
)
```

or from the command line, with the robot already running:

```console
ros2 control load_controller --set-state active manipulator_controller
ros2 control list_controllers
# manipulator_controller  scaled_fjt_controller/ScaledFjtController  active
```

On activation the controller holds the current joint position.

## 4. Send a trajectory from the command line

A trajectory must start at `time_from_start = 0` from the current position. Read it first:

```console
ros2 topic echo --once /joint_states
```

Then send a two-point trajectory (replace `q1 … q6` with the current position, and the second
point with a nearby target):

```console
ros2 action send_goal /manipulator_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      {positions: [q1, q2, q3, q4, q5, q6], velocities: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 0}},
      {positions: [q1+0.2, q2, q3, q4, q5, q6], velocities: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 4}}
    ]}}"
```

The goal finishes with `error_code: 0` (`SUCCESSFUL`) after about 4 seconds.

## 5. Scale the speed

While a trajectory is running, publish an override in percent:

```console
ros2 topic pub --once /speed_ovr std_msgs/msg/Int16 "{data: 50}"   # half speed
ros2 topic pub --once /speed_ovr std_msgs/msg/Int16 "{data: 0}"    # stop on the path
ros2 topic pub --once /speed_ovr std_msgs/msg/Int16 "{data: 100}"  # nominal speed
```

The robot keeps following the same path; only the timing changes. With 50 % the trajectory of
step 4 takes about 8 seconds.

Each override topic keeps its own value and the controller uses their product. A typical use is to
give `/speed_ovr` to the operator and `/safe_ovr` to a safety node (e.g. speed and separation
monitoring): the robot slows down if either of them asks for it.

Values are clamped to [0, 100]; every topic starts at 100 when the controller is configured.

## 6. Use it with MoveIt

Declare the controller in `moveit_controllers.yaml` as a `FollowJointTrajectory` controller and
disable the execution duration monitoring, otherwise MoveIt cancels the trajectories that were
slowed down (see [moveit2#1848](https://github.com/moveit/moveit2/issues/1848)):

```yaml
trajectory_execution:
  execution_duration_monitoring: false

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - manipulator_controller

  manipulator_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
```

Plans executed from RViz or from `MoveGroupInterface::execute()` now run through the controller and
can be slowed down with the override topics.

## 7. Monitor the execution

```console
ros2 topic echo /execution_ratio        # 0 → 1 along the trajectory
ros2 topic echo /scaled_time            # time along the trajectory, in seconds
ros2 topic echo /unscaled_joint_target  # trajectory point at the current scaled time
ros2 topic hz /scaled_time              # should match update_rate
```

`ros2 topic hz /scaled_time` is also a quick check of the control loop: the `max` period should
stay close to the nominal one (2 ms at 500 Hz) even when a long trajectory is sent.

## 8. Debug

The controller logs with the name of the controller. To see the debug messages (the current command
at every cycle, and the first and last three points of each new trajectory), start the controller
manager with:

```console
--ros-args --log-level manipulator_controller:=debug
```

Messages logged from the control loop are written by a background thread; if too many are produced
some are dropped, and a warning reports how many.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `something wrong in interpolation (negative time …)` right after a goal | The first point has `time_from_start > 0`. Make the trajectory start at time 0 from the current position. |
| MoveIt reports `ABORTED` / `CANCELED` after slowing down | Execution duration monitoring is enabled: set `execution_duration_monitoring: false`. |
| `Names are different`, goal aborted with `INVALID_JOINTS` | The goal does not contain exactly the joints of the `joints` parameter. |
| Goal aborted with `Current goal cancelled during deactivate transition` | The controller was deactivated while executing, e.g. by the UR driver `controller_stopper` when the robot program stopped. Check the robot connection. |
| The robot driver drops the connection when a goal arrives (e.g. UR: `Connection to reverse interface dropped`) | The control loop was blocked. This controller does not block on goals of any length; check the real-time setup of the PC (PREEMPT_RT kernel, real-time priority) and other controllers running in the same controller manager. |
