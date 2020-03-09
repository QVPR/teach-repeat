# Building this ROS catkin workspace

Make sure you're in this directory, then run the following commands.

```bash
cd ~/miro/catkin_ws
catkin build
source devel/setup.bash
```

Now you can run demos from this package, like so.

```bash
rosrun miro_demos touch_control.py
```
