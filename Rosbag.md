# Using Rosbag to Record and Replay Data

[Rosbag](https://wiki.ros.org/rosbag) is a useful tool for recording data from ROS and playing it back later. This is useful for example to record sensor data from a robot during a test, which can then be played back later for development without needing the robot anymore.

As an example, we can record some topics from Miro to aid in tuning our image processing. With Miro turned on and the terminal setup with correct ROS ips as described [here](README.md#running-code-remotely-through-ros), you can run the following command to see the list of topics currently being published.

```bash
$ rostopic list -v

Published topics:
 * /miro/sensors/flags [std_msgs/UInt32] 1 publisher
 * /miro/sensors/touch_head [std_msgs/UInt16] 1 publisher
 * /miro/sensors/camr/compressed [sensor_msgs/CompressedImage] 1 publisher
 * /miro/core/detect_motion_l [sensor_msgs/Image] 1 publisher
 * /miro/sensors/touch_body [std_msgs/UInt16] 1 publisher
 ...
 ```

## Recording a Rosbag

If you want to record all available data, you can run either of the following rosbag commands.

```bash
rosbag record -a
# will record until you press Ctrl+c to stop
rosbag record -a --duration=10
# to specify the maximum duration in seconds
...
```

A .bag file will be created in the current directory with name set to the current date and time (such as `2020-03-10-16-28-30.bag`). You can find the details of this bag file by running the following command.

```bash
$ rosbag info 2020-03-10-16-28-30.bag

path:        2020-03-10-16-28-30.bag
version:     2.0
duration:    5.8s
start:       Mar 10 2020 16:28:34.94 (1583821714.94)
end:         Mar 10 2020 16:28:40.72 (1583821720.72)
size:        16.7 MB
messages:    7386
compression: none [22/22 chunks]
...
```

This will show the size and duration of the bag file, as well as the number of types of all topics recorded.

### Recording only Specific Topics

If you only wish to record a certain number of topics to save space, you can specify the topic names when starting recording. For example, to only record the images from Miro's cameras run the following code. **Note:** the cameras are asynchronous so you might not get the same number of images from each camera, even when recording them both for the same amount of time.

```bash
rosbag record /miro/sensors/caml/compressed /miro/sensors/camr/compressed
```

## Playing a Rosbag

Now that you have a rosbag, you can play it back and the ROS topics will communicate with your programs just as they would have when coming from the real robot. **Note:** You'll want to turn off Miro or disconnect from it first, otherwise you will still be receiving messages from Miro while playing back old messages at the same time. Generally, this will cause problems because the recorded messages have older timestamps.

```bash
rosbag play 2020-03-10-16-28-30.bag
```
