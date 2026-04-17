This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_stabilizer_driver:_ DriveTrainDriver

A simple interface to control transmissions.

#### Parameter:

_supported_transfer_cases (string[])_

> Specifies a list with supported transfer cases:
```
DriveTrainDriver:
  supported_transfer_cases:
    - FWD
    - AUTO_4WD
    - MANUAL_LOW_4WD
    - MANUAL_HIGH_4WD
    - LOW_AWD
    - HIGH_AWD
```

_supported_transmissions (string[])_

> Specifies a list with supported transmissions:
```
DriveTrainDriver:
  supported_transmissions:
    - PARK
    - NEUTRAL
    - REVERSE
    - DRIVE
    - OVERDRIVE
    - L1
    - L2
    - L3
    - L4
    - L5
    - L6
    - L7
    - L8
    - L9
    - L10
```


#### Publisher:

_cmd_transmission_state (std_msgs::msg::String)_

> Published the command for new transmission.

_cmd_transfer_case (std_msgs::msg::String)_

> Published the command for new transfer case.

#### Subscriber:

_transmission_state (std_msgs::msg::String)_

> Reads the current transmission.

_transfer_case (std_msgs::msg::String)_

> Reads the current transfer case.
