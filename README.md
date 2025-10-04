# Pinc Open Driver

**pinc_open_driver** provides a ROS 2 hardware interface that allows users to switch between
a mock interface and the actual hardware of Pollen Robotics' open-source
 [Pinc Open Gripper](https://pollen-robotics.github.io/PincOpen/ "Pinc Open Gripper") gripper.
 It includes a simplified URDF based on the Pinc Open Gripper design which allows users to
 incorporate this gripper into their existing robot URDF.
 It also provides a YAML file that sets up two controllers: **position_controllers** and **joint_trajectory_controller**.
The package enables development and testing with either simulated or physical hardware.
It integrates with **ros2_control** and supports easy switching between modes.

<p align="center">
  <img src="docs/hardware.jpeg" alt="Pinc Open Gripper" width="370px" height="370px"/>
  <img src="docs/hardware_ss.png" alt="Pinc Open Gripper Screenshot" width="600px" height="600px"/>
</p>

## Getting Started

Launch one (and only one) of these three launches

### Hardware Launch
* The hardware launch activates the controllers and hardware.

```
clear; ros2 launch pinc_open_driver hardware_launch.py
```

This activates the hardware driver and a RViz viewer.

### Mock Launch
* The mock launch activates the controllers and simulates the hardware.
The default controller activates is the joint_trajectory_controller commands can be sent via command line or using joint_trajectory_controller_gui via rqt.

```
clear; ros2 launch pinc_open_driver display_gripper.py
```

This activates the "mock" loop back interface with a RViz viewer.

### Display Launch
* The display launch loads the robot description and uses the joint_state_publisher_gui to send
commands to the visualized model.
```
clear; ros2 launch pinc_open_driver display_gripper.py
```

> Note: The URDF includes a simplified link model and does not model the exact kinematic 4-bar linkage.


---
## ROS 2 controllers available for use

The provided demo includes set ups for both
- `JointTrajectoryController`(`pinc_open_driver_trajectory_controller`) default
- `JointGroupPositionController` (`pinc_open_driver_position_controller`)
* User can switch between controller using the controller manager plugin in rqt.
<p align="center">
  <img src="docs/controller_manager.png" alt="Pinc Open Gripper" width="700px" height="700px"/>
</p>


By default, the Hardware and Mock launches activates the`pinc_open_driver_trajectory_controller` `JointTrajectoryController` interface.
Commands can be sent via command line, standard action interfaces, and `rqt` gui or our `pinc_open_driver_control_gui`.


### Serial Setup

On initial set up, navigate to the pinc_open_driver directory,
Then, copy the provided `udev` rules below to enable serial communication:

```bash
sudo cp 99-pinc-gripper.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger
```
---

## Helper Scripts

A few simplified logging and plotting scripts are available

* `ros2 run pinc_open_driver pinc_open_driver_control_gui`

    Use sliders and buttons to set goal joint positions and then "send a trajectory"
    > WARNING: No collision checking . Beware of joint limits

* `ros2 run pinc_open_driver pinc_open_driver_monitor`

    This echos joint and controller statuses to terminal while also logging data to the $WORKSPACE_ROOT/log/pinc_logs folder.
    Ctrl-c to terminate and close the log file.

* `ros2 run pinc_open_driver plot_pinc_open_driver_log`

    This plots the last file saved by `pinc_open_driver_monitor`.
<p align="center">
  <img src="docs/monitor_plot.png" alt="Plot of monitor log" width="700px" height="700px"/>
</p>

* `ros2 run pinc_open_driver read_servo`

    Prints data from all accessible servo addresses with a description of each address its length,
    and whether it is read-only or read/write.



