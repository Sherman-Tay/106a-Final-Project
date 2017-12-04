#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
import tf
import numpy as np
from baxter_interface import gripper as robot_gripper
from baxter_interface import settings
import copy

def main():
    #Create listener to /tf
    tf_listener = tf.TransformListener()

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize right arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    right_arm.set_planner_id('RRTstarkConfigDefault')
    right_arm.set_planning_time(20)

    # As long as the node is not killed...
    while not rospy.is_shutdown():
        try:
            # Waits for a number to be entered by the user via the keyboard
            booknum = raw_input('Enter book number corresponding with the book you want to pick:')
        except KeyboardInterrupt:
            print 'Break from raw_input'
            break

        try:
            # set with respect to what frame we would like to get the transforms
            wrt_frame = 'base'
            # set the frame which we would like to transform
            targetMarker = 'ar_marker_'+str(booknum)
            # If both defined frames exist...
            if tf_listener.frameExists(wrt_frame) and tf_listener.frameExists(targetMarker):
                # get the time stamp when both frames were known at the same time
                last_seen = tf_listener.getLatestCommonTime(wrt_frame,targetMarker)
                # read the position and orientation of the target frame in the wrt_frame
                book_pos, book_quat = tf_listener.lookupTransform(wrt_frame,targetMarker,last_seen)

                #First goal pose ----------------------------------------------------------------------
                # position the gripper in the right orientation in front of the shelf
                goal1 = PoseStamped()
                goal1.header.frame_id = "base"

                #x, y, and z position
                goal1_pos = book_pos + qv_mult(book_quat, np.array([0,0,0.1,1]))
                goal1.pose.position.x = goal1_pos[0]
                goal1.pose.position.y = goal1_pos[1]
                goal1.pose.position.z = goal1_pos[2]

                #Orientation as a quaternion
                # Rotate the previous pose by 180 degrees about Y and 90 degrees about Z
                r=0
                p=np.pi
                y=np.pi/2
                q_rot = tf.transformations.quaternion_from_euler(r, p, y)
                goal1_quat = book_quat
                
                # Calculate the new orientation
                goal1_quat = tf.transformations.quaternion_multiply(goal1_quat,q_rot)

                goal1.pose.orientation.x = goal1_quat[0]
                goal1.pose.orientation.y = goal1_quat[1]
                goal1.pose.orientation.z = goal1_quat[2]
                goal1.pose.orientation.w = goal1_quat[3]

                #Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal1)

                #Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                #Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
                right_arm.execute(right_plan)

                # Second goal pose -------------------------------------------------------------------
                # Move towards the book respecting constraints on the movement
                goal2 = PoseStamped()
                goal2.header.frame_id = "base"

                #x, y, and z position
                goal2_pos = book_pos
                goal2.pose.position.x = goal2_pos[0]
                goal2.pose.position.y = goal2_pos[1]
                goal2.pose.position.z = goal2_pos[2]
    
                #Orientation as a quaternion
                goal2.pose.orientation.x = goal1_quat[0]
                goal2.pose.orientation.y = goal1_quat[1]
                goal2.pose.orientation.z = goal1_quat[2]
                goal2.pose.orientation.w = goal1_quat[3]

                #Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal2)

                #Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                # set up orientation constraints in order to prevent collision
                orien_const = OrientationConstraint()
                orien_const.link_name = "right_arm";
                orien_const.header.frame_id = "base";
                orien_const.orientation.x = goal1_quat[0]
                orien_const.orientation.y = goal1_quat[1]
                orien_const.orientation.z = goal1_quat[2]
                orien_const.orientation.w = goal1_quat[3]
                orien_const.absolute_x_axis_tolerance = 0.01;
                orien_const.absolute_y_axis_tolerance = 0.01;
                orien_const.absolute_z_axis_tolerance = 0.01;
                orien_const.weight = 1.0;
                consts = Constraints()
                consts.orientation_constraints = [orien_const]
                right_arm.set_path_constraints(consts)
                rospy.sleep(5.0)

                #Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to goal pose 2: ')
                
                right_arm.execute(right_plan)

                # Activate the GRIPPER
                #Set up the right gripper
                right_gripper = robot_gripper.Gripper('right')
                rospy.sleep(2.0)

                #sets vacuum threshold at 10 out of 18
                right_gripper.set_vacuum_threshold(threshold=10)

                #Initiates suction on the right gripper, setting timeout to an arbitrar
                print('Suction Activated...')
                right_gripper.command_suction(timeout=30.0)
                
                # Third goal pose ------------------------------------------------------------------------
                # Move back a little bit in a straight line
                goal3 = PoseStamped()
                goal3.header.frame_id = "base"

                #x, y, and z position
                goal3_pos = book_pos + qv_mult(book_quat, np.array([0,0,0.15,1]))
                goal3.pose.position.x = goal3_pos[0]
                goal3.pose.position.y = goal3_pos[1]
                goal3.pose.position.z = goal3_pos[2]

                # orientation as a quaternion
                goal3.pose.orientation.x = goal1_quat[0]
                goal3.pose.orientation.y = goal1_quat[1]
                goal3.pose.orientation.z = goal1_quat[2]
                goal3.pose.orientation.w = goal1_quat[3]

                rospy.sleep(5.0)

                # Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal3)

                # Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                # Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to goal pose 3: ')
                right_arm.execute(right_plan)

                #End goal pose ---------------------------------------------------------------------------
                # hard-coded pre defined drop location
                goal4 = PoseStamped()
                goal4.header.frame_id = "base"

                #x, y, and z position
                goal4.pose.position.x = 0.491
                goal4.pose.position.y = -0.548
                goal4.pose.position.z = -0.372

                #Orientation as a quaternion
                goal4.pose.orientation.x = 0.846   
                goal4.pose.orientation.y = 0.532
                goal4.pose.orientation.z = 0.006
                goal4.pose.orientation.w = -0.023

                # remove contraints
                right_arm.clear_path_constraints()

                #Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal4)

                #Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                #Plan a path
                rospy.sleep(5.0)
                right_plan = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to book drop off position: ')
                right_arm.execute(right_plan)
                rospy.sleep(3.0)

                #Ceases suction on the right gripper
                print('Suction Deactivating...')
                right_gripper.stop()
                rospy.sleep(1.0)
                print('stopped')

            else:
                # If not both defin
                ed frames exist...
                print 'The requested AR Tag has not been found.'
        except KeyboardInterrupt:
            print 'Keyboard Interrupt, exiting'
            break

def qv_mult(q1, v1):
    # Function that rotates a vector in a way defined by a quaternion
    return np.dot(tf.transformations.quaternion_matrix(q1),v1)[:3]

if __name__ == '__main__':
    main()
