#!/usr/bin/env python
'''
    This state will be in charge of clear all the missions and transfer the companion computers' data to the GS computer.
'''
import rospy
import smach
from PrintColours import *

class MissionFinished(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['idle', 'shutdown'])
        

    def execute(self, ud):
        rospy.loginfo('[MissionFinished] - MissionFinished state')
        
        
        while not rospy.is_shutdown():            
            #TBD: clear all the missions
            rospy.sleep(5)
            return 'idle'
        return 'shutdown'
