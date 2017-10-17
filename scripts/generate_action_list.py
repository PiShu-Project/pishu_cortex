#!/usr/bin/env python

import cortex_io
import cortex_actions as ca

import cortex_global as cg

def test_face():
    action_list = []

    def fun_mouth_done(c):
        if c['mouth_done'] == True:
            c['mouth_done'] =False
            return True
        else:
            return False

    action_list.append(ca.PiShuWaitUntil(1, 
        lambda c: c['biggest_face_seen_at'] > c['biggest_face_left_at']  ))

    action_list.append(ca.PiShuSay(2, 'I see you')) 
    action_list.append(ca.PiShuWaitUntil(3, fun_mouth_done ))

    action_list.append(ca.PiShuWaitUntil(4, 
        lambda c: c['biggest_face_seen_at'] + 15 < c['counter']  ))

    action_list.append(ca.PiShuSay(5, 'Where are you?')) 
    action_list.append(ca.PiShuWaitUntil(6, fun_mouth_done ))

    action_list.append(ca.PiShuSaveContext(0, 'biggest_face_left_at', 
        'counter', 0)) 

    return action_list


def test1():
    action_list = []

    action_list.append(ca.PiShuWaitFor(1, 2))
    action_list.append(ca.PiShuRandomBranch([2, 3, 4, 6, 11]))
    action_list.append(ca.PiShuSay(5, '2 seconds passed'))
    action_list.append(ca.PiShuSay(5, 'Hi there'))
    action_list.append(ca.PiShuSay(5, 'I will say something in 2 seconds'))

    def fun_mouth_done(c):
        if c['mouth_done'] == True:
            c['mouth_done'] =False
            return True
        else:
            return False
    action_list.append(ca.PiShuWaitUntil(0, fun_mouth_done )) #5

    action_list.append(ca.PiShuSay(7, 'Who')) #6
    action_list.append(ca.PiShuWaitFor(8, 1))
    action_list.append(ca.PiShuSay(9, 'Am'))
    action_list.append(ca.PiShuWaitFor(10, 0.7))
    action_list.append(ca.PiShuSay(0, 'I')) #10

    action_list.append(ca.PiShuSay(12, 'I said something')) #11
    action_list.append(ca.PiShuWaitUntil(13, fun_mouth_done ))
    action_list.append(ca.PiShuSay(14, 'and again'))
    action_list.append(ca.PiShuWaitUntil(15, fun_mouth_done ))
    action_list.append(ca.PiShuRepeatFor([13, 16], 3))
    action_list.append(ca.PiShuSay(0, 'and that is it')) #16

    return action_list

