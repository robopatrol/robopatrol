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

# Run

ROS:

```bash
roscore
```

Turtle Simulation:

```bash
rosrun turtlesim turtlesim_node
```

Robo patrol:

```bash
catkin_make
source devel/setup.bash
rosrun robopatrol main.py
```
