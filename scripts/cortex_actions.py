#!/usr/bin/env python
import sys
from threading import Lock

from time import time 

import random
import math

import cortex_global as cg

import cortex_io

class PiShuAction:

    def __init__(self, next):
        self.next = next

    def reset(self):
        pass

    def act(self):
        return self.next

## Idling functions ##
    
class PiShuWaitFor(PiShuAction):

    start_time = 0

    def __init__(self, next, duration):
        self.next = next
        self.duration = duration

    def act(self):
        if self.start_time == 0:
            self.start_time = time()

        if self.start_time + self.duration < time():
            self.start_time = 0
            return self.next
        else:
            return None

class PiShuWaitUntil(PiShuAction):

    start_time = 0

    def __init__(self, next, condition, duration = None):
        if duration == None:
            duration = float('Inf')
        self.next = next
        self.condition = condition
        self.duration = duration

    def act(self):
        if self.start_time == 0:
            self.start_time = time()
    
        cg.context_lock.acquire()
        res = self.condition(cg.context)
        cg.context_lock.release()

        if res == True or self.start_time + self.duration < time(): 
            self.start_time = 0
            return self.next
        else:
            return None

## Interaction functions ##

class PiShuSay(PiShuAction):

    def __init__(self, next, to_say):
        self.next = next
        self.to_say = to_say

    def act(self):
        cortex_io.say(self.to_say)
        return self.next

## Decision Functions ##

class PiShuRepeatFor(PiShuAction):
    ## returns next[0] a determined number of times and then returns next[1]
    counter = 0

    def __init__(self, next, iterations):
        self.next = next
        self.iterations = iterations
        self.counter = iterations

    def reset(self):
        self.counter = self.iterations

    def act(self):
        if self.counter > 0:
            self.counter = self.counter - 1
            return self.next[0]
        else:
            self.counter = self.iterations
            return self.next[1]

class PiShuRandomBranch(PiShuAction):

    def __init__(self, next):
        self.next = next   

    def act(self):
        return self.next[ random.randrange(len(self.next)) ]    

class PiShuIf(PiShuAction):

    def __init__(self, next, condition):
        self.next = next
        self.condition = condition 

    def act(self):
        cg.context_lock.acquire()
        res = self.condition(cg.context)
        cg.context_lock.release()

        if res == True:
            return self.next[0]
        else: 
            return self.next[1]

## Internal functions

class PiShuSetContext(PiShuAction):

    def __init__(self, next, context_name, context_value, init_value):
        self.next = next   
        self.context_name = context_name
        self.context_value = context_value
        cg.context_lock.acquire()
        cg.context[self.context_name] = init_value
        cg.context_lock.release()

    def act(self):
        cg.context_lock.acquire()
        cg.context[self.context_name] = self.context_value
        cg.context_lock.release()
        return self.next

class PiShuSaveContext(PiShuAction):

    def __init__(self, next, context_name, save_name, init_value):
        self.next = next   
        self.context_name = context_name
        self.save_name = save_name
        cg.context_lock.acquire()
        cg.context[self.context_name] = init_value
        cg.context_lock.release()

    def act(self):
        cg.context_lock.acquire()
        cg.context[self.context_name] = cg.context[self.save_name]
        cg.context_lock.release()
        return self.next

## Debug ##

class PiShuPrint(PiShuAction):

    def __init__(self, next, to_say):
        self.next = next
        self.to_say = to_say

    def act(self):
        print self.to_say
        return self.next


    

