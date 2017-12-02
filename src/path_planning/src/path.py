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
# from tf2_msgs.msg import TFMessage
# import time

# def listener(tf_topic):
#     #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

#     print("Initializing node... ")
#     rospy.init_node("head_cam")
#     print("topic is " + str(tf_topic))

#     rospy.Subscriber(tf_topic,TFMessage,callback)
#     global tf_listener
#     tf_listener = tf.TransformListener()

#     print("Construction of listener completed")

#     #Wait for messages to arrive on the subscribed topics, and exit the node
#     #when it is killed with Ctrl+C
#     rospy.spin()

def main():
    #Create listener to /tf
    tf_listener = tf.TransformListener()

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')
    #rospy.init_node('gripper')
    #rospy.sleep(1.0)

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTstarkConfigDefault')
    right_arm.set_planning_time(20)


    while not rospy.is_shutdown():
        try:
            # Waits for a key input to continue
            booknum = raw_input('Enter book number corresponding with the book you want to pick:')
        except KeyboardInterrupt:
            print 'Break from raw_input'
            break

        try:
            wrt_frame = 'base'
            targetMarker = 'ar_marker_'+str(booknum)
            if tf_listener.frameExists(wrt_frame) and tf_listener.frameExists(targetMarker):
                last_seen = tf_listener.getLatestCommonTime(wrt_frame,targetMarker)
                book_pos, book_quat = tf_listener.lookupTransform(wrt_frame,targetMarker,last_seen)
                print book_pos, book_quat

                #First goal pose ------------------------------------------------------
                goal1 = PoseStamped()
                goal1.header.frame_id = "base"

                #x, y, and z position
                goal1_pos = book_pos + qv_mult(book_quat, np.array([0,0,0.1,1]))
                goal1.pose.position.x = goal1_pos[0]
                goal1.pose.position.y = goal1_pos[1]
                goal1.pose.position.z = goal1_pos[2]

                #Orientation as a quaternion
                # Rotate the previous pose by 180* about Y
                r=0
                p=np.pi
                y=np.pi/2
                q_rot = tf.transformations.quaternion_from_euler(r, p, y)
                goal1_quat = book_quat
                   
                goal1_quat = tf.transformations.quaternion_multiply(goal1_quat,q_rot)  # Calculate the new orientation
                #print goal1_quat, q_rot

                #raw_input('Enter book number corresponding with the book you want to pick:')

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

                # Move towards the book respecting constraints on the movement ----------------------------------------
                goal2 = PoseStamped()
                goal2.header.frame_id = "base"

                #x, y, and z position
                goal2_pos = book_pos #+ qv_mult(book_quat, np.array([0,0,0.01,1]))
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
                right_plan2 = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to goal pose 2: ')
                
                right_arm.execute(right_plan2)
                

                # Activate the GRIPPER
                #Set up the right gripper
                right_gripper = robot_gripper.Gripper('right')
                rospy.sleep(2.0)

                #sets vacuum threshold at 10 out of 18
                right_gripper.set_vacuum_threshold(threshold=10)

                #Initiates suction on the right gripper, setting timeout to an arbitrar
                print('Suction Activated...')
                right_gripper.command_suction(timeout=30.0)
                
                # Move back a little bit in a straight line
                goal3 = PoseStamped()
                goal3.header.frame_id = "base"

                #x, y, and z position
                goal3_pos = book_pos + qv_mult(book_quat, np.array([0,0,0.15,1]))
                goal3.pose.position.x = goal3_pos[0]
                goal3.pose.position.y = goal3_pos[1]
                goal3.pose.position.z = goal3_pos[2]

                #Orientation as a quaternion
                # Rotate the previous pose by 180* about Y
                #r=0
                #p=np.pi
                #y=np.pi/2
                #q_rot = tf.transformations.quaternion_from_euler(r, p, y)
                #goal1_quat = book_quat
                   
                #goal1_quat = tf.transformations.quaternion_multiply(goal1_quat,q_rot)  # Calculate the new orientation
                #print goal1_quat, q_rot

                #raw_input('Enter book number corresponding with the book you want to pick:')

                goal3.pose.orientation.x = goal1_quat[0]
                goal3.pose.orientation.y = goal1_quat[1]
                goal3.pose.orientation.z = goal1_quat[2]
                goal3.pose.orientation.w = goal1_quat[3]

                # orien_const = OrientationConstraint()
                # orien_const.link_name = "right_arm";
                # orien_const.header.frame_id = "base";
                # orien_const.orientation.x = goal1_quat[0]
                # orien_const.orientation.y = goal1_quat[1]
                # orien_const.orientation.z = goal1_quat[2]
                # orien_const.orientation.w = goal1_quat[3]
                # orien_const.absolute_x_axis_tolerance = 0.01;
                # orien_const.absolute_y_axis_tolerance = 0.01;
                # orien_const.absolute_z_axis_tolerance = 0.01;
                # orien_const.weight = 1.0;
                # consts = Constraints()
                # consts.orientation_constraints = [orien_const]
                # right_arm.set_path_constraints(consts)
                # waypoints = []

                # # start with the current pose
                # waypoints.append(right_arm.get_current_pose().pose)

                # # first orient gripper and move forward (+x)
                # wpose = Pose()
                # wpose.orientation.w = 1.0
                # wpose.position.x = goal3.pose.position.x          
                # wpose.position.y = goal3.pose.position.y
                # wpose.position.z = goal3.pose.position.z
                # waypoints.append(copy.deepcopy(wpose))

                # (right_plan, fraction) = right_arm.compute_cartesian_path(
                #              waypoints,   # waypoints to follow
                #              0.01,        # eef_step
                #              0.0)         # jump_threshold
                rospy.sleep(5)

                # Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal3)

                # Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                # Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to goal pose 3: ')
                right_arm.execute(right_plan)

                #End goal pose -------------------------------------------------------------------
                goal_4 = PoseStamped()
                goal_4.header.frame_id = "base"

                #x, y, and z position               #hard-coded pre defined drop location
                goal_4.pose.position.x = 0.491
                goal_4.pose.position.y = -0.548
                goal_4.pose.position.z = -0.372

                #Orientation as a quaternion
                goal_4.pose.orientation.x = 0.846   
                goal_4.pose.orientation.y = 0.532
                goal_4.pose.orientation.z = 0.006
                goal_4.pose.orientation.w = -0.023

                # remove contraints
                right_arm.clear_path_constraints()

                #Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal_4)

                #Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                #Plan a path
                rospy.sleep(5.0)
                right_plan3 = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to book drop off position: ')
                right_arm.execute(right_plan3)
                rospy.sleep(3.0)
                #Ceases suction on the right gripper
                print('Suction Deactivating...')
                right_gripper.stop()
                rospy.sleep(1.0)
                print('stopped')

            else:
                print 'The requested AR Tag has not been found.'
        except KeyboardInterrupt:
            print 'Keyboard Interrupt, exiting'
            break

            

    # #First goal pose ------------------------------------------------------
    # goal_1 = PoseStamped()
    # goal_1.header.frame_id = "base"

    # #x, y, and z position
    # goal_1.pose.position.x = 0.795
    # goal_1.pose.position.y = -0.374
    # goal_1.pose.position.z = 0.279
    
    # #Orientation as a quaternion
    # goal_1.pose.orientation.x = 0.0
    # goal_1.pose.orientation.y = -1.0
    # goal_1.pose.orientation.z = 0.0
    # goal_1.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # right_arm.set_pose_target(goal_1)

    # #Set the start state for the right arm
    # right_arm.set_start_state_to_current_state()

    # #Plan a path
    # right_plan = right_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
    # right_arm.execute(right_plan)

    # #Second goal pose -----------------------------------------------------
    # rospy.sleep(2.0)
    # goal_2 = PoseStamped()
    # goal_2.header.frame_id = "base"

    # #x, y, and z position
    # goal_2.pose.position.x = 0.6
    # goal_2.pose.position.y = -0.3
    # goal_2.pose.position.z = 0.0
    
    # #Orientation as a quaternion
    # goal_2.pose.orientation.x = 0.0
    # goal_2.pose.orientation.y = -1.0
    # goal_2.pose.orientation.z = 0.0
    # goal_2.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # right_arm.set_pose_target(goal_2)

    # #Set the start state for the right arm
    # right_arm.set_start_state_to_current_state()

    # # #Create a path constraint for the arm
    # # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # # orien_const = OrientationConstraint()
    # # orien_const.link_name = "right_gripper";
    # # orien_const.header.frame_id = "base";
    # # orien_const.orientation.y = -1.0;
    # # orien_const.absolute_x_axis_tolerance = 0.1;
    # # orien_const.absolute_y_axis_tolerance = 0.1;
    # # orien_const.absolute_z_axis_tolerance = 0.1;
    # # orien_const.weight = 1.0;
    # # consts = Constraints()
    # # consts.orientation_constraints = [orien_const]
    # # right_arm.set_path_constraints(consts)

    # #Plan a path
    # right_plan = right_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the right arm to goal pose 2: ')
    # right_arm.execute(right_plan)


    # #Third goal pose -----------------------------------------------------
    # rospy.sleep(2.0)
    # goal_4 = PoseStamped()
    # goal_4.header.frame_id = "base"

    # #x, y, and z position
    # goal_4.pose.position.x = 0.6
    # goal_4.pose.position.y = -0.1
    # goal_4.pose.position.z = 0.1
    
    # #Orientation as a quaternion
    # goal_4.pose.orientation.x = 0.0
    # goal_4.pose.orientation.y = -1.0
    # goal_4.pose.orientation.z = 0.0
    # goal_4.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # right_arm.set_pose_target(goal_4)

    # #Set the start state for the right arm
    # right_arm.set_start_state_to_current_state()

    # # #Create a path constraint for the arm
    # # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # # orien_const = OrientationConstraint()
    # # orien_const.link_name = "right_gripper";
    # # orien_const.header.frame_id = "base";
    # # orien_const.orientation.y = -1.0;
    # # orien_const.absolute_x_axis_tolerance = 0.1;
    # # orien_const.absolute_y_axis_tolerance = 0.1;
    # # orien_const.absolute_z_axis_tolerance = 0.1;
    # # orien_const.weight = 1.0;
    # # consts = Constraints()
    # # consts.orientation_constraints = [orien_const]
    # # right_arm.set_path_constraints(consts)

    # #Plan a path
    # right_plan = right_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the right arm to goal pose 3: ')
    # right_arm.execute(right_plan)

def qv_mult(q1, v1):
    return np.dot(tf.transformations.quaternion_matrix(q1),v1)[:3]

if __name__ == '__main__':
    main()
