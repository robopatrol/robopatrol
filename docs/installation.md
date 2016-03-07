# Installation

This instruction assumes that Ubuntu 14.04 is installed and that the 
"restricted," "universe," and "multiverse" components are enabled (if you need 
help on doing this, check out the 
[Ubuntu Guide](https://help.ubuntu.com/community/Repositories/Ubuntu)).

1.  Add ROS package repository to sources

    ```sh
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

2.  Add key to trust the packages from the ROS package repository

    ```sh
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
    ```

3.  Update the package index

    ```sh
    sudo apt-get update
    ```

4.  Install full ROS environment

    ```sh
    sudo apt-get install ros-indigo-desktop-full
    ```

    This includes ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators,
    navigation and 2D/3D perception.

5.  Initialize rosedep

    ```sh
    sudo rosdep init
    rosdep update
    ```

6.  Add ROS environment variables to your terminal session

    ```sh
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

7.  Install rosinstall

    ```sh
    sudo apt-get install python-rosinstall
    ```

8.  Install Turtlebot packages

    ```sh
    sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
    ```


## Test installation

### ROS

Run this command in a terminal to check your ROS installation 

```sh
roscore
```

ROS is installed correctly if you see:

```sh
started core service [/rosout]
```

Close this window or press Ctrl-C to cancel.


### Turtlebot

Start the gazebo turtlebot simulation in a terminal

```sh
roslaunch turtlebot_gazebo turtlebot_world.launch
```

Start 3D visualization tool (rviz) in a second terminal

```sh
roslaunch turtlebot_rviz_launchers view_robot.launch
```

Run the turtlebot teleop application in another terminal

```sh
roslaunch turtlebot_teleop keyboard_teleop.launch
```

Check, if you can move the bot with your keyboard (the terminal window
with the launched teleop application must be active/in focus).

