#!/usr/bin/env python
import rospy
from logikoma_multi_modal.msg import user_input

class ActionTester:
    def __init__(self):
        self.goal_topic = '/keyboard_input'
        self.action_pub = rospy.Publisher(self.goal_topic, user_input, queue_size=1)

    def start(self):
        while not rospy.is_shutdown():
            action = raw_input("Enter an action ")
            #if action in self.valid_actions.keys():
            msg = user_input()
            msg.input = action.strip()
            msg.header.stamp = rospy.get_rostime()
            msg.type = 'keyboard'
            self.action_pub.publish(msg)
            #else:
                #print "Not a valid action"

if __name__=='__main__':
    rospy.init_node('action_tester')
    n = ActionTester()
    n.start()