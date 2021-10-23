# Falcon Controller
This package includes falcon controller for teleoperating mobile robot.

## Dependencies

Please follow the [instruction](../ros_falcon_interface/README.md) to set up the Falcon ROS interface if you haven't.

## Run the controllers

Please run `findfalcons` first to make sure the Falcon Novint is connected properly.

You can specify the published topic names in the command or you could change the default topic name in launch files.

`roslaunch falcon_control falcon_control.launch base_topic:=cmd_vel  ` 

