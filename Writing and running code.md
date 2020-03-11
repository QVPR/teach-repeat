# Writing and Running Code for Miro

This repo already has a catkin workspace initialised for writing ROS code for Miro. There are some example demos available to see how to write ROS nodes for Miro.

## Setting up and Compiling the ROS Workspace

ROS code is stored in a catkin workspace which allows for automatic compilation if setup correctly. A demo catkin workspace is provided in this repo.

```bash
cd ~/miro/catkin_ws
catkin config --merge-devel --no-install --extend ~/mdk/catkin_ws/install
catkin build
source devel/setup.bash
```

You only need to run `catkin config` once, and catkin will remember this configuration. When recompiling code in the workspace, you will normally only need to run `catkin build`. If you have added new packages you may need to run `source devel/setup.bash`. If this is the case you will receive the following message during building: `Note: Workspace packages have changed, please re-source setup files to use them.`

## Running the Demo Programs

Now that your workspace is compiled and you have sourced it, you can run the examples as follows. **Note:** make sure to export the `ROS_MASTER_URI` and/or `ROS_IP` as explained [here](README.md#running-code-remotely-through-ros)

```bash
# this will allow you to control Miro by touching the sensors on its back
# front-middle = forwards, back-middle = backwards, left-middle = turn left, right-middle = turn right
rosrun miro_demos touch_control.py

# this will cause Miro to chase a red ball around - you can run it using only the left or both cameras
rosrun miro_demos ball_detection_single_cam.py
rosrun miro_demos ball_detection_double_cam.py
# at the same time in another terminal, you can run this command to see what Miro is seeing
rosrun image_view image_view image:=/miro/filtered_image
```

These demos show how to access some of Miro's sensors and how to send commands to Miro, such as:

* sending motor commands to Miro on the `/miro/control/cmd_vel` topic
  * as [geometry_msgs/TwistStamped](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) messages
    * `header` contains metadata - most importantly the timestamp so ROS knows when the command was send (if it's too old, it will be ignored)
    * `twist.linear` contains the linear velocity. For Miro, only the `x` component is used (Miro can only drive in the direction it's facing - the x-axis)
    * `twist.angular` contains the angular velocity. For Miro, only the `z` component is used (Miro can only turn about the z-axis, which points up)
* getting the state of the touch sensors on Miro's body on the `/miro/sensors/touch_body` topic
  * as [std_msgs/UInt16](https://docs.ros.org/melodic/api/std_msgs/html/msg/UInt16.html) messages
    * `data` contains one unsigned 16-bit integer. The lower 14 bits each correspond to the state of one touch sensor on Miro's body - 1 if it is touched and 0 if not
* Moving Miro's joints to specific positions on the `/miro/control/kinematic_joints` topic
  * as [sensor_msgs/JointState](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) messages
    * `header` is the same as for the motor commands
    * `name` contains a list of joint names. For Miro's kinematic joints that should be `['tilt','lift','yaw','pitch']`. More explanation [here](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Platform_Geometry)
    * `position` contains a list of position values for each of the joints in radians. **Note:** In this case tilt is a fixed joint, so can't be controlled.
    * `velocity` and `effort` contain joint speeds and forces, which are not required in this case
* Getting images from Miro's cameras on the `/miro/sensors/caml/compressed` or `/miro/sensors/camr/compressed` topics
  * as [sensor_msgs/CompressedImage](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html) messages
  * images are a bit more complicated (especially since these are compressed) so we used [cv_bridge](https://wiki.ros.org/cv_bridge) to assist with converting the images to be used for image processing

A full list of the ROS topics for Miro's sensors and actuators is available [here](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS#sensors/kinematic_joints).

## Making a new Program

To make a new program in the `miro_demos` package, you should create a new python file in the following directory (`catkin_ws/src/miro_demos/src/`). Be sure to mark this file as executable:

```bash
chmod +x catkin_ws/src/miro_demos/src/!new-file!.py
```

Edit the `CMakeLists.txt` file in the miro_demos directory - around line 165 add your new file under `install(PROGRAMS ...)` You should also add any new dependencies into `package.xml` as well as `CMakeLists.txt` Now rebuild the package and you should be able to run your new file.

```bash
catkin build
rosrun miro_demos !new-file!.py
```

## Making a new Package

You may wish to make a new catkin package for your project. This is made easy by a catkin command line tool.

```bash
# in the catkin_ws folder
catkin create pkg -p src !package-name!
```

This will automatically create a new package for you with `package.xml` and `CMakeLists.txt` files. If you now run `catkin build` you will see your new workspace being built.

## Further ROS Information

A detailed beginner's tutorial for ROS is provided [here](https://wiki.ros.org/ROS/Tutorials). Unfortunately it doesn't seem to have been updated super frequently, so a few things might not match exactly with your experience in this workspace. Generally, just substitute `catkin build` for `catkin_make`, `catkin create pkg` for `catkin_create_pkg` and so on... Depending on how interested you are in software development for robots - learning ROS could be very useful.
