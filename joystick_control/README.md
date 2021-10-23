# Joystick Controller
This package includes joystick controller for teleoperating mobile robot. You could control the wheels with the left thumbstick and cameras with the right thumbstick.

## Dependencies

`sudo apt install `

TODO

## Run the controllers

You can specify the published topic names in the command or you could change the default topic name in launch files.

- `roslaunch joystick_control base_topic:=cmd_vel camera_topic:=pitch_yaw  ` 

