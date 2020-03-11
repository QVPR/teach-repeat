# How and When to use the Simulator

The gazebo simulator can be used to test functions you don't wish to test on the robot for some reason. For example, developing behaviours relying on Miro's cliff detection sensors could be dangerous because a bug in the code might cause Miro to drive off the edge of a table.

## Testing code using the simulator

First, make sure the ROS workspace is compiled and setup correctly, as described [here](Writing%20and%20running%20code.md). You will need 3 terminals to run the cliff detection demo. **Note:** the simulation likely won't run very nicely in a VM or in WSL - performance is much better on native Ubuntu.

```bash
# terminal 1
roscore
# terminal 2
gazebo ~/mdk/sim/worlds/tables.world
# terminal 3
rosrun miro_demos cliff_detection.py
# if this produces an error about miro_demos not able to be found, be sure to source catkin_ws/devel/setup.bash in this repo
```

You should see Miro drive along the table and detect the edge, but not reverse enough to avoid falling. We can try and change the parameters to fix this behaviour. Open [cliff_detection.py](catkin_ws/src/miro_demos/src/cliff_detection.py) and change the parameters to the following to increase the evasive action Miro takes.

```python
CLIFF_THRESHOLD = 0.1
REVERSE_TIME = 0.5
SPIN_TIME = 1.0

REVERSE_SPEED = -0.2 #m/s
FORWARDS_SPEED = 0.4 #m/s
SPIN_SPEED = 2 #rad/s
```

Miro should perform a bit more robustly at driving around the table (but will probably still fall off after a while). You can continue to tune these values and/or implement more advanced exploration and edge avoidance behaviours, without fear of damaging your robot.

**Note:** Before trying to transfer software developed in simulation to the real robot, it is important to check that the real world sensors act in a similar way to the simulated sensors. For example, Miro's cliff sensors will likely be significantly more noisy that in simulation. More information is provided [here](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Husbandry_MIROapp_Controller_page#Cliff%20sensor%20sensitivity). To deal with this you will likely want to change the `CLIFF_THRESHOLD` paramter in [cliff_detection.py](catkin_ws/src/miro_demos/src/cliff_detection.py).

## When is the Simulator not Useful

There are still limitations with the simulator that prevent it being used to develop all types of robot software. For example, the visual scenes rendered by the simulator will not be as realistic as scenes in the real world. In this case, a better approach would be to record data from the real world using [rosbag](Rosbag.md) and tune algorithms and approaches based on this data.
