# Apriltags Package for ROS2

## Running pose estimation with eye-tracker's camera

1) ```ros2 run tobii_glasses_pkg tobii_glasses.pyros2```
2) ```ros2 run apriltags_pkg pose_estimation_eyetracker```
3) ```ros2 run apriltags_pkg visual_interface```

## Executing test with dummy publisher and robot: 20/12

1) ```ros2 launch apriltags_pkg dummy_eyetracker_launch.launch```
2) ```ros2 launch apriltags_pkg apriltags_launch.launch```