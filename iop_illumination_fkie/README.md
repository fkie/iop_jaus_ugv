This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_illumination_fkie:_ Illumination

Control the lights of a robot.

#### Parameter:

_illuminations (list_, (Default: [])

> A list of {key: value} with supported lights and their states. Keys are:

    - head_lights
    - left_turn_signal
    - right_turn_signal
    - running_lights
    - brake_lights
    - backup_lights
    - visible_light_source
    - ir_light_source
    - variable_light_1
    - variable_light_2
    - variable_light_3
    - variable_light_4
    - high_beams
    - parking_lights
    - fog_lights
    - hazard_lights

> Supported states are `ON`, `OFF`, `0` (not supported), `1` (supported - unknown state). For each supported light a publicher for commands and a subscriber for current state are created.


#### Publisher:

_illuminator/cmd_`key` (std_msgs::Bool)_

> Command for supported light.

#### Subscriber:

_illuminator/`key` (std_msgs::Bool)_

> State for supported light.
