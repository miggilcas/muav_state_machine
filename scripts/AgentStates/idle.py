#!/usr/bin/env python

import rospy
import smach
from PrintColours import *

class Idle(smach.State):
    def __init__(self,pub,autopilot):#modify, common_data
        smach.State.__init__(
            self, outcomes=['gcs_connection', 'shutdown'])
        self.pub = pub
        self.autopilot = autopilot

    def execute(self, ud):
        rospy.loginfo('[Idle] - Idle state')
        
        # transition to gcs_connection state
        while not rospy.is_shutdown():            
            if self.autopilot == "px4":
                rospy.loginfo(CBLUE2 +'There are %d connections to the topic of PX4'+CEND, self.pub.get_num_connections())# Modify
            if self.autopilot == "dji":
                rospy.loginfo(CBLUE2 +'There are %d connections to the topic of DJI'+CEND, self.pub.get_num_connections())
            if self.pub.get_num_connections() >= 1:
                return 'gcs_connection'
            rospy.sleep(2)
        return 'shutdown'
