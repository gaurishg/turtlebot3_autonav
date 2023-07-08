#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import random

positions = [
    Point(-7.25, -3.3, 0.0),
    Point(-5.75, -3.3, 0.0),
    Point(-6.3, 4.5, 0.0),
    Point(-0.5, 4.5, 0.0),
    Point(1.0, 1.5, 0.0),
    Point(5.0, 4.0, 0.0),
    Point(7.0, 4.5, 0.0),
    Point(6.0, -5.0, 0.0)
]

quaternions = [
    Quaternion(0.0, 0.0, 0.0, 1.0), # UP
    Quaternion(0.0, 0.0, -1.0, 0.0), # DOWN
    Quaternion(0.0, 0.0, 0.7, 0.7), # LEFT
    Quaternion(0.0, 0.0, -0.7, 0.7), # RIGHT
]


def active_callback():
    rospy.loginfo("Received the goal")

def feedback_callback(feedback: MoveBaseFeedback):
    # print("feedback")
    # rospy.loginfo(feedback)
    ...

def done_callback(status: int, result: MoveBaseResult):
    if status == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached")
    elif status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
        rospy.loginfo("Goal cancelled")
    elif status == GoalStatus.ABORTED:
        rospy.loginfo("Goal aborted")
    rospy.loginfo(f"{result=}")


def main():
    rospy.init_node("qibitech_turtlebot3_house")

    # Set initial pose of robot using information from Gazebo
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    odom: Odometry = rospy.wait_for_message("/odom", Odometry) # type: ignore

    # Generate msg
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose = odom.pose.pose
    rospy.sleep(1)
    pub.publish(msg)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.sleep(1)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = random.choice(positions)
        goal.target_pose.pose.orientation = random.choice(quaternions)

        client.send_goal(goal, done_callback, active_callback, feedback_callback)
        finished = client.wait_for_result()

        if not finished:
            rospy.logerr("Action server is not available")
        else:
            rospy.loginfo(client.get_goal_status_text())


if __name__ == '__main__':
    main()
    
    