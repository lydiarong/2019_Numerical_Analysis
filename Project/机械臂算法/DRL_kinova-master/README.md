# DRL_kinova

## camera calibration:
TODO

## bring up kinova jaco2:
```
roslaunch kinova_bringup kinova_robot.launch kinovarobotType:=j2n6s300
```
## control the real kinova arm using MoveIt:
```
roslaunch j2n6s300_moveit_config j2n6s300_demo.launch
```
## control the virtual kinova arm using MoveIt:
```
roslaunch j2n6s300_moveit_config j2n6s300_virtual_robot_demo.launch
```
## bring up usbcam and apriltag:
```
roslaunch kinova_description kinova_with_camera.launch
```
## hand_eye_calibration:
```
roslaunch easy_handeye kinova_usbcam_calibration.launch
```
If you want to use it in the simulation, edit the launch file, set the          parameter<name="freehand_robot_movement" value="false" />
## using the result of hand eye calibration:
```
roslaunch easy_handeye kinova_camera_tf_publisher.launch
```
Make sure the ~/.ros/easy_handeye/easy_handeye_eye_on_base.yaml file, the content is the parameter that kinova_usbcam_calibration.launch calculate.
