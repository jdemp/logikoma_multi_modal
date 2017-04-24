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
    return transformations.euler_from_quaternion(quat)


def rotate_around_z(degrees, quat):
    euler = quat_to_euler(quat)
    return euler_to_quat(euler[0],euler[1],math.radians(degrees))


class Logikoma:
    def __init__(self):
        self.goal_topic = '/goal'
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pose = Pose()
        self.stopped = True
        self.auto = False

    def go_forward(self, action):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.pose.position.x + action
        goal.target_pose.pose.position.y = self.pose.position.y
        goal.target_pose.pose.position.z = self.pose.position.z
        goal.target_pose.pose.orientation = self.pose.orientation
        self.move_base.send_goal(goal)

    def navigate_to_point(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
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
        goal.target_pose.pose.position.x = self.pose.position.x - 1
        goal.target_pose.pose.position.y = self.pose.position.y
        goal.target_pose.pose.position.z = self.pose.position.z
        goal.target_pose.pose.orientation = self.pose.orientation
        self.move_base.send_goal(goal)

    def turn(self, action):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.pose.position.x + 0.5
        if action == 'left':
            goal.target_pose.pose.position.y = self.pose.position.y - 0.5
            goal.target_pose.pose.orientation = rotate_around_z(-90,self.pose.orientation)
        else:
            goal.target_pose.pose.position.y = self.pose.position.y + .5
            goal.target_pose.pose.orientation = rotate_around_z(90,self.pose.orientation)
        self.move_base.send_goal(goal)

    def rotate(self, action):
        pass

    def stop(self):
        #may need to add current pose

        goal = MoveBaseGoal()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.pose.position.x
        goal.target_pose.pose.position.y = self.pose.position.y
        goal.target_pose.pose.position.z = self.pose.position.z
        goal.target_pose.pose.orientation = self.pose.orientation
        self.move_base.send_goal(goal)
        self.stopped = True

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
        else:
            print "I can't complete that action"

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





if __name__ == '__main__':
    rospy.init_node('Logikoma')
    n = Logikoma()
    n.run()