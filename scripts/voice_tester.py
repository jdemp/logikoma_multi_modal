#!/usr/bin/env python
from os import environ, path

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import logging


logging.getLogger('requests').setLevel(logging.CRITICAL)

MODELDIR = "../models"
#DATADIR = "../models/data"

# Create a decoder with certain model
config = Decoder.default_config()
config.set_string('-hmm', path.join(MODELDIR, 'en-us/en-us'))
#config.set_string('-lm', path.join(MODELDIR, 'en-us/en-us.lm.bin'))
config.set_string('-dict', path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))
config.set_string('-kws', '../models/keywords.list')
decoder = Decoder(config)

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
decoder.start_utt()
while True:
    buf = stream.read(1024)
    decoder.process_raw(buf, False, False)
    if decoder.hyp() != None:
        words = []
        [words.append(seg.word) for seg in decoder.seg()]
        print words
        decoder.end_utt()
        decoder.start_utt()

#print ('Best hypothesis segments: ', [seg.word for seg in decoder.seg()])