#!/usr/bin/env python

#from multiprocessing import Process
import subprocess
import time
import datetime
import sys
import os
import signal
import rospy
import psutil


# change this
# linux home dir (~/) doesn't work :(
username = "remi"

class RecordMap():

    def __init__(self):
	duration = 15  # [s]
        bagfile = self.startRecording(duration)
	time.sleep(1)

	rospy.loginfo("finished recording to " + bagfile)


    # @param : int duration [seconds]
    # Returns the name of the bag file
    # A bag file (see rosbag documentation) is a container of recorded messages
    def startRecording(self,duration):
	timestamp = time.time()
	now = datetime.datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d_%H-%M-%S")

	bagfilePath = "/home/" + username + "/catkin_ws/src/robopatrol/"
        bagfile = bagfilePath + "robopatrol_" + now + ".bag"

	rospy.loginfo("Starting the recording process...")	
	subprocess.call("rosbag record --duration=" + str(duration) + " -O " + bagfile + " scan tf", shell=True)

	return bagfile


    # not being used, just for debugging
    def __killParentAndChildrenProcesses(self,ppid):
	parent = psutil.Process(ppid)

	for child in parent.children(recursive=True):
	    child.kill()
	parent.kill()


    # not being used, just for debugging
    def __killProcess(self,proc):
	try:
	    count = 0
            while proc.is_alive() and count < 10:
	        proc.terminate()
	   	count = count + 1
		time.sleep(1)

	    if proc.is_alive():
                raise Excetption("cannot terminate the process " + proc.name)
	    else:
		return 0
        except:
	    print("cannot terminate the process " + proc.name)
            return -1
	

if __name__ == "__main__":
    RecordMap()
