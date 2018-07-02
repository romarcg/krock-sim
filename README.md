## krock-sim

Code and files to simulate *krock* robot in webots simulator. The goal is to perform several runs focusing in locomotion to extract data of the traversability of the robot.

> The `.deb` file of the webots simulator is not included. Check the cyberbotics webpage to download such file (~500mb).

> Verify the `docker-compose.yml` file before running the container. Ideally a joystick would help when testing the implemented ROS controller. Use rostopic to bypass this requirement.

> A compiled version of the docker container is up in dockerhub.

> This container image assumes an nvidia gpu is present. If this is not the case, the docker file needs to be changed appropriately (e.g. intel).



## How to krock

### Installation

Follow the steps on the github repository to compile the docker image. Remember to dowloand the webots binary.

### General description of the current architecture

Webots development environment uses "controllers" to manage simulated robots or models in the simulated environment. Currently, we have already implemented a controller that interfaces with webots krock model and ROS middleware.

An only-webots controller and a ros-webots controller are in `~/krock` folder.

Currently the pipeline is compose of two parts: the simulator (webots) and the ROS side (nodes).

Ideally, the simulator will only run the simulated robot (sic) in a desired map. A ROS node will spawm the robot on a specific pose and send a movement command (e.g. move forward). Then the ros node will store (e.g. in rosbags) the movement of the robot in such map with such command. This process will repeat several times to store a large dataset of robot's traversability.

### webots

Once inside the container, run the simulator using `webots`. Enter your credentials.
Open (`File > Open World`) the test world `krock2.wbt` in `~/krock/krock2_ros/worlds`. This world file includes instructions to load the map (store as a matrix inside such file) and the robot model (which also calls the krock2_ros controller).

> Ideally, this process needs to be automatized so that a roslaunch can call for the simulator to open different maps (previously generated).


### ROS side

When launching webots as described in the last section an error will appear indicating that a connection to ROS master was not found. This is due to the roscore not initialized.

In another terminal (inside the container) execute `roscore`. Then in webots interface restart the simulation (Ctrl + Shift + r). The error will disappear and the controller (from webots side) is ready to receive commands and publish topics with robot state information.

In another terminal (inside the container) execute `rostopic list` for a quick overview of the current available topics (published by the krock2_ros webots controller).

Currently, I have implemented to ways to interface with the robot, the simplest one is using the `/krock/gait_chooser` topic with 0 -- 3 value. Another one is using the `/joy` topic that ideally needs a joystick to control the robot. The former option only chooses the gait and moves forward, the latter will allow to increase speed and also make turns.

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

A very simple python script in `simulate_traversability` package is present to give an idea on how to read data from webots simulator.

This ROS side also needs to be automatized. Ideally, it must:

i. launch the webots simulatore with a desired world and spawn the robot at a random pose,
ii. send a movement command (e.g. move forward at certain speed),
iii. record the movement (pose, torques feedback and touch sensors) published by the controller,
iv. once a certain period of time has passed (e.g. 20s) store all the data (e.g. a rosbag)
v. respawm the robot on another random pose and repeat from ii. This could also be changed to be a single run experiment (the simplest the better)
