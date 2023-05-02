#!/usr/bin/env python

import rospy
import smach
from PrintColours import *

from multimaster_msgs_fkie.srv import DiscoverMasters

# debug variables
n_uavs = 2

# function to discover the masters
def discover_masters():
    rospy.wait_for_service('/master_discovery/list_masters')
    try:
        masters = rospy.ServiceProxy('/master_discovery/list_masters', DiscoverMasters)
        resp = masters()
        return resp.masters
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

class Idle(smach.State):
    def __init__(self):#modify, common_data
        smach.State.__init__(
            self, outcomes=['UAVs_monitoring','shutdown'])
        

    def execute(self, ud):
        rospy.loginfo('[Idle] - Idle state')
        
        # transition to X state
        while not rospy.is_shutdown():            
            masters_list = discover_masters()
            rospy.loginfo(CBLUE+"The lenght of master's list is: {}, so there is/are {} companion computer(s)".format(len(masters_list),len(masters_list)-1)+CEND)
            # compare the lenght of the master list with the number of uavs
            # if the lenght is the same, then transition to the next state
            # if the lenght is different, then stay in this state
            if len(masters_list) == n_uavs: # this can control the multimaster state 
                return 'UAVs_monitoring'
            rospy.sleep(2)
        return 'shutdown'
