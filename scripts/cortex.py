#!/usr/bin/env python
import sys
from threading import Lock

from time import sleep

import rospy

import cortex_global as cg

import cortex_io
import cortex_actions as ca
import generate_action_list

def main():
    rospy.init_node('cortex', anonymous=False)

    cortex_io.init()

    cg.context_lock.acquire()
    cg.context['counter'] =  0
    cg.context_lock.release()

    action_list = generate_action_list.test_face()
    action_pointer = 0

   

    # add a counter to the context, that is incremented as each time step


    ## Main Loop    

    while not rospy.is_shutdown():
        cg.context_lock.acquire()
        cg.context['counter'] =  cg.context['counter'] + 1
        cg.context_lock.release()
        next = action_list[action_pointer].act()
        while next != None:
            action_pointer = next
            next = action_list[action_pointer].act()
        #print cg.context['biggest_face_left_at']
        #print cg.context['counter']
        sleep(0.1)

if __name__ == '__main__':
    main()  




