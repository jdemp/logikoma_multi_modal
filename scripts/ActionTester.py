#!/usr/bin/env python
import rospy
from logikoma_multi_modal.msg import action_output

class ActionTester:
    def __init__(self):
        self.valid_actions = {'go straight':'go straight', 'continue':'continue', 'turn left':'turn left',
                              'turn right':'turn right'}
        self.goal_topic = '/user_goal'
        self.action_pub = rospy.Publisher(self.goal_topic, action_output, queue_size=1)

    def start(self):
        while not rospy.is_shutdown():
            action = raw_input("Enter an action ")
            if action in self.valid_actions.keys():
                msg = action_output()
                msg.action = action
                msg.header.stamp = rospy.get_rostime()
                self.action_pub.publish(msg)
            else:
                print "Not a valid action"

if __name__=='__main__':
    rospy.init_node('action_tester')
    n = ActionTester()
    n.start()