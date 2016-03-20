import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import random
from math import radians


class RoboPatrol():

    def __init__(self):
        """
        Initializes the TurtleBot
        """
        rospy.init_node('robopatrol_move')
        
    	rospy.loginfo("To stop RoboPatrol CTRL + C")

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        mode = input("Select operation mode: a (Autonmous), u (User controlled):")

        if mode == 'a':
            self.drive_autonomous()
        elif mode == 'u':
            while not rospy.is_shutdown():
                speed = input("Speed: ")
                distance = input("Distance: ")
                turn = input("Turn: ")
                self.move(speed, distance, turn)
        else:
            rospy.loginfo("Invalid input: " + mode)


    def move(self, speed, distance, turn):
        """Moves the TurtleBot by specified speed, distance and turn angle
        speed -- the speed in meters / second
        distance -- the distance it will drive TODO provide unit
        turn -- the angle it will turn in degrees
        """
        move_cmd = Twist()
        
        move_cmd.linear.x = speed
        move_cmd.linear.y = 0
        move_cmd.linear.z = 0

        move_cmd.angular.x = 0
        move_cmd.angular.y = 0
        # TODO make sure 'turn' is between 0 and 360
        move_cmd.angular.z = radians(turn)

        t0 = rospy.rostime.get_time()
        current_distance = 0
        rate = rospy.Rate(100);

        while current_distance < distance:
            self.cmd_vel.publish(move_cmd)
            t1 = rospy.rostime.get_time()
            current_distance = abs(speed * (t1 - t0))
            rate.sleep()

        self.cmd_vel.publish(Twist())

    def drive_autonomous(self):
        """
        Lets the TurtleBot drive around by itself, distance and turn radius are
        currently determined randomly.
        """
        rospy.loginfo("Driving by myself")
        while not rospy.is_shutdown():
            speed = 0.5 # meter per seconds
            distance = random.randint(1, 5) # TODO in which unit is this measured?
            turn = random.randint(0, 45) # in degrees, will be converted to radians
            self.move(speed, distance, turn)

    #if bump data is received, process here
    #data.bumper: LEFT (0), CENTER (1), RIGHT (2)
    #data.state: RELEASED(0), PRESSED(1)
    # copied from: https://canvas.harvard.edu/courses/7567/pages/getting-started-ros-turtlebot-sensors-and-code
    # might be useful to detect objects
    def processBump(data):
        global bump
        if (data.state == BumperEvent.PRESSED):
            bump = True
        else:
            bump = False
        rospy.loginfo("Bumper Event")
        rospy.loginfo(data.bumper)
        
    def shutdown(self):
        rospy.loginfo("Stop RoboPatrol")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

