# Mobile_Robot_ROS_Control_Interfaces
This package implements ROS teleoperation interfaces for controlling the mobile robots. Interfaces include the keyboard, joystick, steering wheel G29, and Novint Falcon.

![cover](demo/cover.jpg)

## Dependencies

`git clone --recurse-submodules git@github.com:hiro-wpi/Mobile_Robot_ROS_Control_Interfaces.git ` 

If you do not need any of these interfaces, you could simply delete its corresponding folders to avoid build error (Falcon folder will cause make error without properly set up). 

- To use Keyboard, TODO
- To use Joystick, run `jstest-gtk` to get the proper input channel (e.g. js0), and set it in `joystick_control/joystick_control.launch`.
- To use Falcon Novint, please follow the [falcon interface README](https://github.com/ZhuoyunZhong/ros_falcon_interface/blob/d0d968b7dade353171fab427fd1fe01244846a6b/README.md) to install libfalcon and for other setups. Test to see if the Falcon is ready by `findfalcons`. If any errors show up, please find the corresponding solutions in README.
- To use G29 Steering Wheel force feedback, please follow the [g29 force feedback README](https://github.com/kuriatsu/ros-g29-force-feedback/blob/61eef1c78dc9863b9c1a119f75338cb167d6c03c/README.md) for device configuration. Also, run `jstest-gtk` to get the proper input channel (e.g. js1), and set it in `g29_steering_wheel_control/g29_control.launch`.

## Run the controllers

You can specify the published topic names in the command or you could change the default topic name in launch files.

This package includes keyboard and mouse controller for teleoperating mobile robot. You could control the wheels with keyboard and cameras with mouse.

- `roslaunch keyboard_control TODO  ` 

- `roslaunch joystick_control joystick_control.launch base_topic:=cmd_vel camera_topic:=pitch_yaw` 
- `roslaunch falcon_control falcon_control.launch base_topic:=cmd_vel`  
- `roslaunch g29_steering_wheel_control g29_control.launch base_topic:=cmd_vel`

For more details, please refer to the README files in each folder.

