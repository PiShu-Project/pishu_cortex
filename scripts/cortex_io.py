#!/usr/bin/env python
import sys
from threading import Lock

import rospy
from std_msgs.msg import String, Int16, Bool
from pishu_msgs.msg import MultiFaces, FacePosition

import cortex_global as cg

pubs = {}

def say(str_to_say):
    pubs['say'].publish(str_to_say)

def mouth_done_callback(data):
    cg.context_lock.acquire()
    cg.context['mouth_done'] = True
    cg.context_lock.release()

def biggest_face_detection_callback(data):
    cg.context_lock.acquire()
    cg.context['biggest_face_pos'] = [data.x, data.y, data.w]
    cg.context['biggest_face_seen_at'] = cg.context['counter']
    cg.context_lock.release()

def init():

    pubs['say'] = rospy.Publisher("mouth/chatter", String, queue_size = 5)
    pubs['behave'] = rospy.Publisher("cortex/behavior", Int16, queue_size = 5)

    rospy.Subscriber("/mouth/done", Bool, mouth_done_callback)
    rospy.Subscriber("/eye/biggest_face", FacePosition, biggest_face_detection_callback)

    cg.context['mouth_done'] = False
    cg.context['biggest_face_pos'] = [0, 0, 0]
    cg.context['biggest_face_seen_at'] = -1


    
