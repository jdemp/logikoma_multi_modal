#!/usr/bin/env python
import rospy, rospkg
from logikoma_multi_modal.msg import user_input, action_output
#from logikoma_multi_modal.scripts import SpeechMapper
from SpeechMapper import SpeechMapper

class Logikoma_User_Input:

    def __init__(self):
        pkg_path = rospkg.RosPack().get_path('logikoma_multi_modal')
        self.MODELDIR = pkg_path + "/models/"
        self.use_speech = rospy.get_param('/logikoma_user_input/use_speech', False)
        print str(self.use_speech)
        self.use_gesture = rospy.get_param('use_gesture', False)
        self.use_keyboard = rospy.get_param('use_keyboard', False)
        self.goal_topic = rospy.get_param('/goal_topic', '/goal')
        self.goal_pub = rospy.Publisher(self.goal_topic, action_output, queue_size=1)
        self.input_history = []
        self.number_of_inputs = 0
        self.current_inputs = []
        self.input_mappers = {}

    def decide_action(self):
        if len(self.current_inputs)==1:
            msg = self.current_inputs.pop()
            if msg.type == 'speech':
                action = self.input_mappers['speech'].process(msg.input)
            else:
                action = 'none'
            msg = action_output()
            msg.action = action
            msg.header.stamp = rospy.get_rostime()
            self.goal_pub.publish(msg)

    def process(self, msg):
        self.input_history.append(msg)
        self.current_inputs.append(msg)

    def start(self):
        if self.use_speech:
            speech_topic = rospy.get_param('/speech_topic', '/speech_recog')
            self.input_mappers['speech'] = SpeechMapper(True, False, False)
            self.input_mappers['speech'].set_static_mapping(self.MODELDIR + 'speech_mapping.mapping')
            rospy.Subscriber(speech_topic, user_input, callback=self.process)
            self.number_of_inputs += 1
        if self.use_keyboard:
            keyboard_topic = rospy.get_param('/keyboard_topic', '/keyboard_input')
            self.input_mappers['keyboard'] = SpeechMapper(True, False, False)
            self.input_mappers['keyboard'].set_static_mapping(self.MODELDIR + 'speech_mapping.mapping')
            rospy.Subscriber(keyboard_topic, user_input, callback=self.process)

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