#!/usr/bin/env python
import rospy
from baxter_interface import gripper as robot_gripper
from baxter_interface import settings
#from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

#Set up the right gripper
right_gripper = robot_gripper.Gripper('right')

#sets vacuum threshold at 15 out of 18
print('Setting vacuum threshold to 18/18')
right_gripper.set_vacuum_threshold(threshold=18)

#Initiates suction on the right gripper, setting timeout to an arbitrar
print('Suction Activated...')
right_gripper.command_suction(timeout=1000)
rospy.sleep(15.0)


#Ceases suction on the right gripper
print('Suction Deactivated...')
right_gripper.stop()
rospy.sleep(10.0)
print('Done with cycle')

#create if loop we are at start positon, init suction, only releasing when arr. final position