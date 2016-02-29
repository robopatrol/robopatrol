# Robo-Patrol

## Laws

1. A robot may not injure a human being or, through inaction, allow a human being to come to harm.
2. A robot must obey orders given it by human beings except where such orders would conflict with the First Law.
3. A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.


# Technologies & Tools

* Hardware: TurtleBot 2 (http://www.turtlebot.com/)
* TurtleBot runs on ROS (http://wiki.ros.org/)
* Programming Language: Python (IDE: PyCharm)
* Documentation: Google Drive


# Install
1. Install ROS for Ubuntu (experimental instructions available for OSX): http://wiki.ros.org/indigo/Installation/Ubuntu
2. In addition, you will need to install the following debs for TurtleBot (as described [here] (http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation#turtlebot.2BAC8-Tutorials.2BAC8-indigo.2BAC8-Debs_Installation.Debs_Installation) )

 ```
 sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
 ```
3. For convenience, you may wish to source your setup.sh script from your .bashrc so that your environment is ready as soon as you log in. e.g.

 ```
 echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
 ```

# Run

ROS:

```bash
roscore
```

Turtle Simulation:

```bash
rosrun turtlesim turtlesim_node
```

[Stage Simulation] (http://wiki.ros.org/turtlebot_stage/Tutorials/indigo/Bring%20up%20TurtleBot%20in%20stage):

```roslaunch turtlebot_stage turtlebot_in_stage.launch```

[Gazebo Simulation] (http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Gazebo%20Bringup%20Guide):

```roslaunch turtlebot_gazebo turtlebot_world.launch  ```

Robo patrol:
(change into your workspace folder, e.g. ```cd /RoboPatrol ``` )

```bash
catkin_make
source devel/setup.bash
rosrun robopatrol main.py
```


