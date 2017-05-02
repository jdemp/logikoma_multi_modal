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
        self.use_keyboard = rospy.get_param('/logikoma_user_input/use_keyboard', False)
        print str(self.use_keyboard)
        self.goal_topic = rospy.get_param('/goal_topic', '/goal')
        self.goal_pub = rospy.Publisher(self.goal_topic, action_output, queue_size=1)
        self.input_history = []
        self.number_of_inputs = 0
        self.current_inputs = []
        self.input_mappers = {}
        self.invalid_multi_modal = [('turn left', 'turn right'), ('turn right', 'turn left'), ('hard left', 'hard right'),
                                    ('hard right', 'hard left'), ('turn left', 'hard right'), ('hard right', 'turn left'),
                                    ('hard left', 'turn right'), ('turn right', 'hard left')]


    def get_msg(self, msg):
        if msg.type == 'speech':
            action = self.input_mappers['speech'].process(msg.input)
        elif msg.type == 'keyboard':
            action = self.input_mappers['keyboard'].process(msg.input)
        else:
            action = 'none'
        action_msg = action_output()
        action_msg.action = action
        return action_msg

    def decide_action(self):
        if len(self.current_inputs)==1:
            msg = self.current_inputs.pop()
            msg = self.get_msg(msg)
            msg.header.stamp = rospy.get_rostime()
            if not msg.action == 'No Action':
                self.goal_pub.publish(msg)
        elif len(self.current_inputs)==2:
            second_input = self.current_inputs.pop()
            first_input = self.current_inputs.pop()
            action_one = self.get_msg(first_input)
            action_two = self.get_msg(second_input)
            if action_one.action == action_two.action and not action_two.action == 'No Action':
                self.goal_pub.publish(action_two)
            else:
                action_pair = (action_one.action, action_two.action)
                if self.invalid_multi_modal.count(action_pair) == 0:
                    self.goal_pub.publish(action_one)
                    self.goal_pub.publish(action_two)

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
            self.number_of_inputs += 1

        if self.number_of_inputs >0:
            print "Running with " + str(self.number_of_inputs) + " user inputs"
            rate = rospy.Rate(.2)
            while not rospy.is_shutdown():
                self.decide_action()
                rate.sleep()


if __name__=='__main__':
    rospy.init_node('logikoma_user_input')
    n = Logikoma_User_Input()
    n.start()