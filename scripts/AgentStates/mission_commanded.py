#!/usr/bin/env python

import rospy
import smach
from mavros_msgs.msg import State
from std_msgs.msg import UInt8
from PrintColours import *


class MissionCommanded(smach.State):
    def __init__(self,autopilot, uav_id):#
        smach.State.__init__(
            self, outcomes=['mission_running', 'shutdown'])
        self.autopilot = autopilot
        self.uav_id = uav_id

    def execute(self, ud):
        rospy.loginfo('[MissionCommanded] - MissionCommanded state')
        rospy.set_param("/uav_{}_sm/autopilot".format(self.uav_id),"MissionCommanded")
        # transition to mission_running state
        while not rospy.is_shutdown():
            if self.autopilot == "px4":
                state_msg = rospy.wait_for_message("/uav_{}/mavros/state".format(self.uav_id), State)
                if state_msg.armed:
                    rospy.loginfo(CBLUE+"Vehicle with PX4 armed, Mission started"+CEND)
                    return 'mission_running'
                else:
                    rospy.loginfo(CBLUE+"Vehicle with PX4 disarmed"+CEND)
                    rospy.sleep(1)
            elif self.autopilot == "dji":
                state_msg = rospy.wait_for_message("/uav_{}/dji_osdk_ros/flight_status".format(self.uav_id), UInt8)
                if state_msg.data == 2: # In flight
                    rospy.loginfo(CBLUE+"Vehicle DJI in mission, Mission started"+CEND)
                    return 'mission_running'
            # if (True):# Modify
            #     rospy.sleep(14)
            #     return 'mission_running'
            
            
        return 'shutdown'
