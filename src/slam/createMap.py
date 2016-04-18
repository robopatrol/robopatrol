#!/usr/bin/env python

import subprocess
import time
import time
import rospy


class CreateMap():

    def __init__(self):
	self.saveMap()


    def playBackRecording(self,bagfile):
	if not bagfile == "":
	    self.__setROSparams()
            time.sleep(1)
	    subprocess.call("rosbag play --clock " + bagfile, shell=True)


    def __setROSparams(self):
	subprocess.call("rosparam set use_sim_time true", shell=True)


    def startGMapping(self):
	subprocess.call("rosrun gmapping slam_gmapping", shell=True)


    def saveMap(self):
	subprocess.call("rosrun map_server map_saver", shell=True)
	

if __name__ == "__main__":
    CreateMap()
