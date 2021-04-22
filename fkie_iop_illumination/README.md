This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_illumination:_ Illumination

Control the lights of a robot.

#### Parameter:

> Pairs of {key: value} with supported lights and their states. Keys are:

```
IlluminationService:
  head_lights: no
  left_turn_signal: no
  right_turn_signal: no
  running_lights: no
  brake_lights: no
  backup_lights: no
  visible_light_source: no
  ir_light_source: no
  variable_light_1: no
  variable_light_2: no
  variable_light_3: no
  variable_light_4: no
  high_beams: no
  parking_lights: no
  fog_lights: no
  hazard_lights: no
```

> Supported states are `ON`, `OFF`, `no` (not supported), `yes` (supported - unknown state). For each supported light a publisher for commands and a subscriber for current state are created.


#### Publisher:

_illuminator/cmd_`key` (std_msgs::Bool)_

> Command for supported light.

#### Subscriber:

_illuminator/`key` (std_msgs::Bool)_

> State for supported light.
