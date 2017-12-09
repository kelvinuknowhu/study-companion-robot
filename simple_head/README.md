# Simple Head
## A ROS controller for the Cornell Dynamixel tablet head robot 
[Original GitHub Repo](https://github.com/guyhoffman/simple_head)

### Launch
```bash
$ roslaunch study_pose.launch
```

### Customize poses
You can add poses in the `poses.yaml` config file. After you add a new pose, you should be able to send the new pose name on the `/goto_pose` topic. There are four degrees of freedom available for you to define a pose:
- `head_roll`
- `head_tilt`
- `head_pan`
- `neck_tilt`
