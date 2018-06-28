## krock-sim

Code and files to simulate *krock* robot in webots simulator. The goal is to perform several runs focusing in locomotion to extract data of the traversability of the robot.

> The `.deb` file of the webots simulator is not included. Check the cyberbotics webpage to download such file (~500mb).

> Verify the `docker-compose.yml` file before running the container. Ideally a joystick would help when testing the implemented ROS controller. Use rostopic to bypass this requirement.

> A compiled version of the docker container is up in dockerhub.

> This container image assumes an nvidia gpu is present. If this is not the case, the docker file needs to be changed appropriately (e.g. intel).
