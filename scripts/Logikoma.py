#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from logikoma_multi_modal.msg import action_output
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import math


def euler_to_quat(roll,pitch,yaw):
    return transformations.quaternion_from_euler(roll,pitch,yaw)


def quat_to_euler(quat):
    return transformations.euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])


def rotate_around_z(degrees, quat):
    euler = quat_to_euler(quat)
    return euler_to_quat(euler[0],euler[1],math.radians(degrees))


#need to transform coordinates from odom frame to map frame before sending the action to the robot
#look at sending the action to the /goal topic instead of the action client
class Logikoma:
    def __init__(self):
        self.goal_topic = '/user_goal'
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pose = Pose()
        self.stopped = True
        self.auto = False

    def go_forward(self, action):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1 #3 meters
        goal.target_pose.pose.orientation.w = 1.0 #go forward
        self.move_base.send_goal(goal)

    def navigate_to_point(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = self.pose.position.z
        goal.target_pose.pose.orientation = self.pose.orientation
        self.move_base.send_goal(goal)

    def go_back(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -1
        goal.target_pose.pose.orientation.w = 1.0 #go forward
        self.move_base.send_goal(goal)

    def turn(self, action):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = .5
        if action == 'left':
            goal.target_pose.pose.position.y = .5
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        else:
            goal.target_pose.pose.position.y = -.5
            goal.target_pose.pose.orientation.z = -.7071
            goal.target_pose.pose.orientation.w = .7071
        self.move_base.send_goal(goal)

    def rotate(self, action):
        pass

    def done(self):
        pass

    def process(self,user_goal):
        # create list of goals and which method to call each method will parse the action
        if user_goal.action =='stop':
            self.move_base.cancel_goal()
            self.stopped = True
        elif user_goal.action == 'continue':
            pass
        elif user_goal.action == 'search':
            self.auto = True
        elif user_goal.action == 'done':
            self.done()
        elif user_goal.action == 'go back':
            self.move_base.cancel_goal()
            self.go_back()
        elif user_goal.action == 'turn left':
            self.move_base.cancel_goal()
            self.turn('left')
        elif user_goal.action == 'turn right':
            self.move_base.cancel_goal()
            self.turn('right')
        elif user_goal.action == 'go straight':
            self.move_base.cancel_goal()
            self.go_forward(1)
        else:
            print "I can't complete that action"

        success = self.move_base.wait_for_result(rospy.Duration(60))
        if success:
            print "I did it"
        else:
            print "Failed to complete action"

    def update_pose(self,msg):
        self.pose = msg.pose.pose

    def on_shutdown(self):
        #save action history to file
        print "Ending"

    def run(self):
        rospy.Subscriber(self.goal_topic, action_output, callback=self.process)
        rospy.Subscriber('/odom', Odometry, callback=self.update_pose)
        rospy.on_shutdown(self.on_shutdown)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            continue




if __name__ == '__main__':
    rospy.init_node('Logikoma')
    n = Logikoma()
    n.run()