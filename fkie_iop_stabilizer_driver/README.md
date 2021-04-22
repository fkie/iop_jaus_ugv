This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_stabilizer_driver:_ StabilizerDriver

A simple interface to control a set of flipper of a robot by velocity effort or position. The flippers can be controlled by _sensor_msgs::JointState_ or _std_msgs::Float64MultiArray_ messages. Both messages are published, so be sure to subscribe only one of them.

#### Parameter:

_max_up_angle (double_, (Default: 1.5708)

> The maximum supported angle for _all_ flipper. This value is only used for capabilities report.

_max_down_angle (double_, (Default: -1.5708)

> The minimum supported angle for _all_ flipper. This value is only used for capabilities report.

_joint_names (list_ Default: [])

> Specifies a list with joint names. This is important to get the position of flipper, e.g.:
```
StabilizerDriver:
  joint_names:
    - actuator_0_joint
    - actuator_1_joint
```


#### Publisher:

_cmd_joint_states (sensor_msgs::JointState)_

> Published the position or velocity commands for joints specified by _joint_names_ parameter.

_flipper_velocity_controller/command (std_msgs::Float64MultiArray)_

> Published velocity commands as an array. The count corresponds to the count of elements specified in _joint_names_ parameter.

#### Subscriber:

_joint_states (sensor_msgs::JointState)_

> Reads for joints specified by _joint_names_ parameter the positions of the flippers and reports them to controller.
