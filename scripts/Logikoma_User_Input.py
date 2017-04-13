#!/usr/bin/env python
import rospy
from logikoma_multi_modal.msg import user_input

class Logikoma_User_Input:

    def __init__(self):
        self.use_voice = rospy.get_param('/logikoma_user_input/use_voice', True)
        print str(self.use_voice)
        self.use_gesture = rospy.get_param('use_gesture', False)
        self.use_keyboard = rospy.get_param('use_keyboard', False)
        self.goal_topic = rospy.get_param('/goal_topic', '/goal')
        self.goal_pub = rospy.Publisher(self.goal_topic, user_input, queue_size=1)
        self.input_history = []
        self.number_of_inputs = 0
        self.current_inputs = []

    def decide_action(self):
        if len(self.current_inputs)==1:
            msg = self.current_inputs.pop()
            self.goal_pub.publish(msg)

    def process(self, msg):
        self.input_history.append(msg)
        self.current_inputs.append(msg)

    def start(self):
        if self.use_voice:
            voice_topic = rospy.get_param('/speech_topic', '/speech_recog')
            rospy.Subscriber(voice_topic, user_input, callback=self.process)
            self.number_of_inputs += 1

        if self.number_of_inputs >0:
            print "Running with " + str(self.number_of_inputs) + " user inputs"
            rate = rospy.Rate(5)
            while not rospy.is_shutdown():
                self.decide_action()
                rate.sleep()


if __name__=='__main__':
    rospy.init_node('logikoma_user_input')
    n = Logikoma_User_Input()
    n.start()