#!/usr/bin/env python


# Robo-Patrol: check battery status of kobuki base and netbook

import rospy
import roslib
from kobuki_msgs.msg import SensorState
from smart_battery_msgs.msg import SmartBatteryStatus    # needed for the netbook battery
import math

class BatteryStatus():

    lowBaseBattery = False
    lowNetbookBattery = False
    basePreviousBatteryLevel = 100
    netbookPreviousBatteryLevel = 100
    baseMaxCharge = 100
    # The maximum electric charge of the kobuki base can be determined by running:
    # rostopic echo /mobile_base/sensors/core 
    # check the "battery" value.


    def getBaseMaxCharge(self):
	return baseMaxCharge


    def __init__(self):
        rospy.init_node('batteryStatus')
	
	# monitor Kobuki's power and charging status if an event occurs (low battery, charging, not charging) 
	rospy.Subscriber("/mobile_base/sensors/core",SensorState,self.sensorPowerEventCallback)

	# monitor netbook's battery power
	rospy.Subscriber("/laptop_charge/",SmartBatteryStatus,self.netbookPowerEventCallback)


    def batteryLevel(self,data):
	# kobuki's batttery value tends to bounce up and down 1 constantly so only report if difference greater than 1
	if (math.fabs(int(data.battery) - basePreviousBatteryLevel) > 2):
	    rospy.loginfo("Kobuki's battery is now: " + str(round(float(data.battery) / float(getBaseMaxCharge()) * 100)) + "%")
	    basePreviousBatteryLevel = int(data.battery)
	    returnValue = "Kobuki's battery is now: " + str(round(float(data.battery) / float(getBaseMaxCharge()) * 100)) + "%"
	    
	return returnValue

	def isCharging(self, data):
	if (int(data.charger) == 0):
	    if (charging):
		rospy.loginfo("Stopped charging")
		charging = False
		returnValue = "Stopped charging"
	else:
	    if (not charging):
		rospy.loginfo("Pumping petrol at the station")
		charging = True
		returnValue = "Pumping petrol at the station"

	return returnValue
	
	def batteryStatus(self, data):
	if (round(float(data.battery) / float(getBaseMaxCharge()) * 100) < 25) :
	    if (not lowBattery):
		rospy.loginfo("Kobuki battery is low")
		lowBattery = True
		returnValue = "Kobuki battery is low"
	elif (round(float(data.battery) / float(getBaseMaxCharge()) * 100) > 35):
	    # the logic of not using the same value (e.g. 25) for both the battery is low & battery is fine is that it'll leave and
     	    # immediatly return for more power.
	    if (lowBattery):
		rospy.loginfo("Kobuki battery is fine")
		lowBattery = False
		returnValue = "Kobuki battery is fine"

	return returnValue
	

# refactor to own class?
    def netbookPowerEventCallback(self,data):
	# has the netbook's power level changed?
	if (int(data.percentage) != netbookPreviousBatteryLevel):
	    rospy.loginfo("Notebook's battery is now: " + str(data.percentage) + "%")
	    netbookPreviousBatteryLevel = int(data.percentage)
	# is the netbook's power low?
	if (int(data.percentage) < 25):
            lowNetbookBattery = True
	elif (int(data.percentage) > 35):
	    # the logic of not using the same value (e.g. 25) for both the battery is low & battery is fine is that it'll leave and
            # immediatly return for more power.
	    lowNetbookBattery = False

	# complete list of information available in the /laptop_charge/ thread
	#print(data)
