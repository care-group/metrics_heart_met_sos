#!/usr/bin/env python3

# standard libraries
import rospy
import rospkg
import speech_recognition as sr
from queue import Queue
from threading import Thread
from datetime import datetime
import usb.core
import usb.util
import time
import sys

# internal classes
from log import Log
from tuning import Tuning

# standard messages
from std_msgs.msg import String, Int32

# custom messages
# none

# constants and parameters
# none

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospy.init_node('metrics_speech')

        self.ros_pub_direction_raw = rospy.Publisher('/metrics_speech/direction_raw', Int32, queue_size=10)

        # set up classes
        # none

        # set up HSR
        # none

        # instance variables
        self.rospack = rospkg.RosPack()
        self.rel_path = self.rospack.get_path('metrics_speech')

        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = False
        self.r.energy_threshold = 2000
        # self.r.dynamic_energy_adjustment_ratio = 1.1
        self.audio_queue = Queue()

        # ready
        self.logger.log_great('Ready.')

        # loop
        self.run()

    def run(self):
        listen_thread = Thread(target=self.listen_worker)
        listen_thread.daemon = True
        listen_thread.start()

        while not rospy.is_shutdown():
            rospy.sleep(1)

    def listen_worker(self):
        recognize_thread = Thread(target=self.recognize_worker)
        recognize_thread.daemon = True
        recognize_thread.start()
        with sr.Microphone() as source:
            try:
                while True and not rospy.is_shutdown():
                    self.logger.log('Listening...')
                    timed_out = False
                    try:
                        audio = self.r.listen(source, timeout=1, phrase_time_limit=10)
                    except sr.WaitTimeoutError:
                        timed_out = True
                    if not timed_out:
                        now = datetime.now()
                        now = now.strftime('%d-%m-%Y_%H-%M-%S')
                        path = self.rel_path + '/src/audio/' + now + '.wav'
                        with open(path, 'wb') as file:
                            file.write(audio.get_wav_data())
                        self.audio_queue.put(audio)
            except KeyboardInterrupt:
                pass

        self.audio_queue.join()
        self.audio_queue.put(None)
        recognize_thread.join()

    def recognize_worker(self):
        self.logger.log('Started worker thread.')
        while True:
            audio = self.audio_queue.get()
            if audio is None:
                break

            utterance = ''
            try:
                # utterance = self.r.recognize_google_cloud(audio, language='en-GB')
                # utterance = self.r.recognize_whisper(audio, language='English', model='base')
                pass
            except sr.UnknownValueError:
                self.logger.log_warn('No results from Google Cloud STT.')
            log = 'Utterance:' + utterance
            self.logger.log(log)
            words = utterance.split()
            if len(words) > 0:
                hotword = words[0]
                hotword = hotword.lower()
                hotword = hotword[0:5]
                if hotword == 'david':
                    utterance = utterance.lstrip('google')
                    msg = String()
                    msg.data = utterance
                    self.ros_pub_rasa_utterance_internal.publish(msg)

            self.logger.log('Terminated worker thread.')
            self.audio_queue.task_done()

if __name__ == '__main__':
    m = Main()