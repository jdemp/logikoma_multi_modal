#!/usr/bin/env python
from os import environ, path

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import logging
import rospy
from logikoma_multi_modal.msg import user_input


class SpeechRecog:
    def __init__(self):
        #DATADIR = "../models/data"
        MODELDIR = "logikoma_multi_modal/models"
        self.pub = rospy.Publisher('/speech_recog', user_input, queue_size=1)

        # Create a decoder with certain model
        self.config = Decoder.default_config()
        self.config.set_string('-hmm', path.join(MODELDIR, 'en-us/en-us'))
        #config.set_string('-lm', path.join(MODELDIR, 'en-us/en-us.lm.bin'))
        self.config.set_string('-dict', path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))
        self.config.set_string('-kws', 'logikoma_multi_modal/models/keywords.list')
        self.decoder = Decoder(self.config)
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)

    def publish(self,user_in):
        msg = user_input()
        msg.input = user_in
        msg.type = 'speech'
        msg.header.stamp = rospy.get_rostime()
        self.pub.publish(msg)

    def listen(self):
        self.decoder.start_utt()
        while True:
            buf = self.stream.read(1024)
            self.decoder.process_raw(buf, False, False)
            if self.decoder.hyp() != None:
                words = []
                [words.append(seg.word) for seg in self.decoder.seg()]
                self.publish(words[0])
                self.decoder.end_utt()
                self.decoder.start_utt()

if __name__ == '__main__':
    rospy.init_node('speech_recog')
    n = SpeechRecog()
    n.listen()
