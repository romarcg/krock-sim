## General description of the current architecture

Webots development environment uses "controllers" to manage simulated robots or models in the simulated environment. Currently, we have already implemented a controller that interfaces with webots krock model and ROS middleware.

An only-webots controller and a ros-webots controller are in ~/krock folder.

Currently the pipeline is compose of two parts: the simulator (webots) and the ROS side (nodes).

Ideally, the simulator will only run the simulated robot (sic) in a desired map. A ROS node will spawm the robot on a specific pose and send a movement command (e.g. move forward). Then the ros node will store (e.g. in rosbags) the movement of the robot in such map with such command. This process will repeat several times to store a large dataset of robot's traversability.

### Non-docker considerations

If a non-docker setup is preferred or inevitable the current state of the repository may change according to the preferred development tools and paths. Webots' part is in `krock` folder and runs independently of the ROS side. However, the current webots robot controller waits until it detects a rosmaster running. ROS' part is in `catkin_ws_src` folder and its content must be (1)

> (1) source code can be place in other folder but it needs to be declared using other building tools. If you are not familiar with ROS `catkin-tools` custom features avoid this to reduce the extra steps.

### Webots

Run the simulator using `webots` in a terminal. Enter your credentials. Open (`File > Open World`) the test world `krock2_camera.wbt` in `~/krock/krock2_ros/worlds`. This world file includes instructions to load the map (stored as a matrix inside such file) and the robot model (which also calls the krock2_ros controller).

> Ideally, this process needs to be automatized so that a `roslaunch` can call for the simulator to open different maps (previously generated).


> `webots` toolbars contains all the needed commands to play, pause and reset the simulation. Through the menu options an editor and compilation tools can be enabled in case the controller need to be changed or recompiled.


### ROS side

When launching `webots` as described in the last section an error will appear indicating that a connection to ROS master was not found. This is due to `roscore` not being initialized.

In another terminal execute `roscore`. Then in webots interface restart the simulation (Ctrl + Shift + r). The error will disappear and the controller (from `webots` side) is ready to receive commands and publish topics with robot state information.

In another terminal execute `rostopic list` for a quick overview of the current available topics (published by the `krock2_ros` `webots` controller).

Currently, I have implemented three ways to interface with the robot, the simplest one is using the `/krock/gait_chooser` topic with `0`-- `3` value. Another one is using the `/joy` topic that ideally needs a joystick to control the robot. The former option only chooses the gait and moves forward, the latter will allow to increase speed and also make turns. Even if joystick is not present, `rostopic pub` can be used to send similar commands.

Using `/kgrock/gait_chooser`:

```
rostopic pub /krock/gait_chooser webots_ros/Int8Stamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
data: 1"
```

A third way to control the robot is though a one show message send to to the topic `/krock/manual_control_input`. Such message is composed of four numbers (mode, gait, frontal freq, lateral freq), the first one chooses the mode (`1, 0`), the second the gait (`0, 1, 2, 3`) and the third and four are floats (`[-1.0, 1.0]`) to indicate the frequency of reading the joint tables on the specific table gait. These frequency numbers act as speed controllers, frontal freq for full forward `(1.0)` or full backwards `(-1.0)` and the lateral freq for rotations.

This example sets the manual mode using the normal gait at "full" forward speed:

```
rostopic pub /krock/manual_control_input webots_ros/Float64ArrayStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
data:
- 1
- 1
- 1.0
- 0.0"
```

A very simple python script in `simulate_traversability` package is present to give an idea on how to read data from webots simulator. This script can be executed using `rosrun` tool.

> A ros node called webots_supervisor_tools.py` in the ros package `simulate_traversability` shows how to access the set of ros services exposed by the webots controller. Use this node as a base to interface with the krock robot. For example, to activate the front camera of the robot, a service must be called. Such service will start publishing the camera image in the respective topic.

> Remember to build the packages `simulate_traversability` and `webots_ros`. **source** the result so to be able to execute the script (which must be executable via chmod).

This ROS side also needs to be automatized. Ideally, it must:

i. launch the webots simulator with a desired world and spawn the robot at a random pose,
ii. send a movement command (e.g. move forward at certain speed),
iii. record the movement (pose, torques feedback and touch sensors) published by the controller,
iv. once a certain period of time has passed (e.g. 20s) store all the data (e.g. a rosbag)
v. respawm the robot on another random pose and repeat from ii. This could also be changed to be a single run experiment (the simplest the better)
