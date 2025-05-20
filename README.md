# scaled_fjt_controller

A ros2 basic implementation of the scaled_fjt_controller in [scaled_follow_joint_trajectory_controller](https://github.com/JRL-CARI-CNR-UNIBS/scaled_follow_joint_trajectory_controller).

This package implements a [ros2_controller](https://control.ros.org/master/index.html) as a FollowJointTrajectory Action with the possibility to dynamically scale the execution velocity in runtime.

This controller inherits directly from the [joint_trajectory_controller/JointTrajectoryController](https://control.ros.org/iron/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).

## Installation
If you are already using [ros2_control](https://github.com/ros-controls/ros2_control) with the [ros2_controllers](https://github.com/ros-controls/ros2_controllers), no additional modules are required. 
Just
```console
cd <to/your/ros2_ws>/src
git clone https://github.com/paolofrance/scaled_fjt_controller
cd ..
colcon build --symlink-install
```
source your workspace.

## Usage
In your ros2_controllers configuration file, set the following

```
controller_manager:
  ros__parameters:

    manipulator_controller:
      type: "scaled_fjt_controller/ScaledFjtController"

manipulator_controller:
  ros__parameters:
    joints:
      - J1
      - J2
      - J3
      - J4
      - J5
      - J6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    open_loop_control: true
    constraints:
      goal_time: 0.0
    speed_ovr_topics:
      topics: ["speed_ovr"]
      policy: "MINIMUM" 
    state_publish_rate: 100.0
    action_monitor_rate: 20.0

```

Now you can bring up your robot packages and load this controller. 

To scale the velocity, on a new terminal, simply type  
```console
ros2 topic pub /speed_ovr std_msgs/msg/Int16 {"data: 10"}
```
The {"data: 10"} value can take any integer in the range [0-100] and represents a percentage of the nominal velocity of the goal trajectory.

An example of usage with Moveit! is available [here](https://github.com/paolofrance/ros2_fanuc_interface?tab=readme-ov-file#trajectory-execution-velocity-scaling).


### Options

The speed scaling value is read from a list of topics:
```
    speed_ovr_topics:
      topics: ["speed_ovr"]
      policy: "MINIMUM" 
```

- ```topics``` is the list of topics the controller will listen to. The expected topic type is ```std_msgs/msg/Int16``` with scaling values between 0 and 100.

- ```policy``` is the method to compute the overall scaling value when ```topics``` contains more than one element. Available policies are:
  - ```MULTIPLY```: computes the product of the scaling values read from each topic
  - ```MINIMUM```: computes the minimum of the scaling values read from each topic
  - ```MAXIMUM```: computes the maximum of the scaling values read from each topic
  - ```AVERAGE```: computes the average of the scaling values read from each topic

**Default:** 

- speed_ovr_topics.topics = ["speed_ovr","safe_ovr"]
- speed_ovr_topics.policy = "MULTIPLY"


### Troubleshooting
- Moveit! cancels the trajectory execution when it requires too much than expected (due to a strong scaling). To avoid this behavior, go to `moveit_controllers.yaml` file of your cell moveit package and write
```
trajectory_execution:
  execution_duration_monitoring: false
```
See [this link](https://github.com/moveit/moveit2/issues/1848) for more info.



