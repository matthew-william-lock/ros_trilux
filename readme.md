# ros_trilux

[![CI](https://github.com/matthew-william-lock/ros_fluorometer/actions/workflows/main.yml/badge.svg)](https://github.com/matthew-william-lock/ros_fluorometer/actions/workflows/main.yml) [![license](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

ROS driver and messages for [TriLux Fluorometer](https://chelsea.co.uk/products/trilux-algae/).

<p align="center">
  <img src="https://user-images.githubusercontent.com/53016036/233779781-3ab6d705-e14c-4507-8651-119d6b3c5260.png" width="100%">
</p>

## Launching

```bash
roslaunch ros_trilux trilux_measure.launch
```

Launch parameters:
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `driver_port` | `string` | `/dev/ttyUSB0` | Serial port to connect to |
| `driver_baud` | `int` | `9600` | Serial port baudrate |

## Topics
| Topic | Type | Description |
| --- | --- | --- |
| `core/trilux/measurement` | `ros_trilux_msgs::Measurement` | Measurements from the TriLux |