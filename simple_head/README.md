# Simple Head
## A ROS controller for the Cornell Dynamixel tablet head robot 
[Original GitHub Repo](https://github.com/guyhoffman/simple_head)

You should be able to run the nodes in this package by simply launching the `simple_head.launch` file. Depends on [dynamixel_controller](http://wiki.ros.org/dynamixel_controllers)

```
roslaunch simple_head.launch
```

### Customize poses
You can add poses in the `poses.yaml` config file. After you add a new pose, you should be able to send the new pose name on the `/goto_pose` topic. There are four degrees of freedom available for you to define a pose:
- `head_roll`
- `head_tilt`
- `head_pan`
- `neck_tilt`

### Publish poses in Terminal
You request a pose by sending a `simple_head/PoseCommand` message to the `/goto_pose` topic. The message has two fields: a ROS duration (the time it should take to go to the pose) and a string for the pose. All the available commands can be found in `config/poses.yaml`.

<<<<<<< Updated upstream
For example, this will request pose `up_left` with a movement duration of three seconds:
```rostopic pub --once /goto_pose simple_head/PoseCommand -- '[3.0,0.0]' 'up_left'```
=======
For example, this will request pose `up_left` with a movement duration of 3 seconds:
```
rostopic pub -1 /goto_pose simple_head/PoseCommand -- '[3.0,0.0]' 'up_left'
```
>>>>>>> Stashed changes


### Common issues
- Make sure that you have the right USB port (usualy `/dev/ttyUSB0` but might be different on your machine).

