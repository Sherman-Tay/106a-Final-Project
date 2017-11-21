#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import tf
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

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTstarkConfigDefault')
    right_arm.set_planning_time(10)


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
                book_pos, book_quat = tf_listener.lookupTransform(targetMarker,wrt_frame,last_seen)
                print book_pos, book_quat

                #First goal pose ------------------------------------------------------
                goal = PoseStamped()
                goal.header.frame_id = "base"

                #x, y, and z position
                goal.pose.position.x = book_pos[0]
                goal.pose.position.y = book_pos[1]
                goal.pose.position.z = book_pos[2]+1
    
                #Orientation as a quaternion
                goal.pose.orientation.x = book_quat[0]
                goal.pose.orientation.y = book_quat[0]
                goal.pose.orientation.z = book_quat[0]
                goal.pose.orientation.w = book_quat[0]

                #Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal)

                #Set the start state for the right arm
                right_arm.set_start_state_to_current_state()

                #Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
                right_arm.execute(right_plan)
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
    # goal_3 = PoseStamped()
    # goal_3.header.frame_id = "base"

    # #x, y, and z position
    # goal_3.pose.position.x = 0.6
    # goal_3.pose.position.y = -0.1
    # goal_3.pose.position.z = 0.1
    
    # #Orientation as a quaternion
    # goal_3.pose.orientation.x = 0.0
    # goal_3.pose.orientation.y = -1.0
    # goal_3.pose.orientation.z = 0.0
    # goal_3.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # right_arm.set_pose_target(goal_3)

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

if __name__ == '__main__':
    main()
