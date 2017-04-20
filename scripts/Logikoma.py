import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from logikoma_multi_modal.msg import action_output
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf


def euler_to_quat(self, euler):
    return tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)


def quat_to_euler(self,quat):
    return tf.transformations.euler_from_quaternion(quat)


class Logikoma:
    def __init__(self):
        self.goal_topic = '/goal'
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pose = Pose()
        self.stopped = True

    def go_forward(self, action):
        success = True
        while success:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'base_link'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.pose.position.x +1
            goal.target_pose.pose.position.y = self.pose.position.y
            goal.target_pose.pose.position.z = self.pose.position.z
            goal.target_pose.pose.orientation = self.pose.orientation
            self.move_base.send_goal(goal)
            success = self.move_base.wait_for_result(rospy.Duration(15))
        self.stop()

    def navigate_to_point(self, x, y):
        pass

    def go_back(self):
        pass

    def turn(self, action):
        pass

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
        success = self.move_base.wait_for_result(rospy.Duration(15))
        self.stopped = True

    def done(self):
        pass

    def process(self,user_goal):
        # create list of goals and which method to call each method will parse the action
        pass

    def update_pose(self,msg):
        self.pose = msg.pose.pose

    def run(self):
        rospy.Subscriber(self.goal_topic, action_output, callback=self.process)
        rospy.Subscriber('/odom', Odometry, callback=self.update_pose)


if __name__ == '__main__':
    rospy.init_node('Logikoma')
    n = Logikoma()
    n.run()