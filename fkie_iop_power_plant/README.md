This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## fkie_iop_power_plant:_ PowerPlantManager

A list with powerplants and their capabilities. Format: ID.POWERPLANT.PARAMETER.VALUE

Currently reports on the status of one battery. It should be configured like:

```
    power_plants:
      - 1.battery.voltage.58
```


#### Parameter:

_power_plants/ID/voltage (int_ (Default: 0))

> The maximal voltage of the battery.

#### Publisher:

> None

#### Subscriber:

_powerplant\_`ID`/voltage (std_msgs::Float32)_

> Current voltage.

_powerplant\_`ID`/capacity_percent (std_msgs::Int8)_

> Percent of maximum.
