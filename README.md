See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:

[iop_stabilizer_driver_fkie: StabilizerDriver](#iop_stabilizer_driver_fkie-stabilizersriver)  
[iop_illumination_fkie: Illumination](#iop_illumination_fkie-illumination)  


## _iop_stabilizer_driver_fkie:_ StabilizerDriver

A simple interface to control a set of flipper of a robot by velocity effort or position. The flippers can be controlled by _sensor_msgs::JointState_ or _std_msgs::Float64MultiArray_ messages. Both messages are published, so be sure to subscribe only one of them.

#### Parameter:

_max_up_angle (double_, (Default: 1.5708)

> The maximum supported angle for _all_ flipper. This value is only used for capabilities report.

_max_down_angle (double_, (Default: -1.5708)

> The minimum supported angle for _all_ flipper. This value is only used for capabilities report.

_joint_names (list_ Default: [])

> Specifies a list with joint names. This is important to get the positon of flipper.


#### Publisher:

_cmd_joint_states (sensor_msgs::JointState)_

> Published the position or velocity commands for joints specified by _joint_names_ parameter.

_flipper_velocity_controller/command (std_msgs::Float64MultiArray)_

> Published velocity commands as an array. The count corresponds to the count of elements specified in _joint_names_ parameter.

#### Subscriber:

_joint_states (sensor_msgs::JointState)_

> Reads for joints specified by _joint_names_ parameter the positions of the flippers and reports them to controller.


## _iop_illumination_fkie:_ Illumination

Control the lights on the robot.

#### Parameter:

_illuminations (list_, (Default: [])

> A list of {key: value} with supported lights and their states. Keys are [head_lights, left_turn_signal, right_turn_signal, running_lights, brake_lights, backup_lights, visible_light_source, ir_light_source, variable_light_1, variable_light_2, variable_light_3, variable_light_4, high_beams, parking_lights, fog_lights, hazard_lights]. Supported states are `ON`, `OFF`, `0` (not supported), `1` (supported - unknown state). For each supported light a publicher for commands and a subscriber for current state are created.


#### Publisher:

_illuminator/cmd_`key` (std_msgs::Byte)_

> Command for supported light.

#### Subscriber:

_illuminator/`key` (std_msgs::Byte)_

> State for supported light.

