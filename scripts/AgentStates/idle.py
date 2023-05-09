#!/usr/bin/env python

import rospy
import smach
from PrintColours import *
from std_msgs.msg import String

class Idle(smach.State):
    def __init__(self,pub,autopilot,uav_id):#modify, common_data
        smach.State.__init__(
            self, outcomes=['gcs_connection', 'shutdown'])
        self.pub = pub
        self.autopilot = autopilot
        self.uav_id = uav_id

    def execute(self, ud):
        rospy.loginfo('[Idle] - Idle state')
        airframe_pub = rospy.Publisher("/uav_{}_sm/com/airframe_type".format(self.uav_id), String, queue_size=10)
        
        mission_state_pub = rospy.Publisher("/uav_{}_sm/com/mission_state".format(self.uav_id), String, queue_size=10)
        if self.autopilot == "px4":
            airframe = self.autopilot + "/vtol"
        if self.autopilot == "dji":
            airframe = self.autopilot + "/M210"
        # transition to gcs_connection state
        while not rospy.is_shutdown():      
            airframe_pub.publish(airframe)
            mission_state_pub.publish("idle")      
            if self.autopilot == "px4":
                rospy.loginfo(CBLUE2 +'There are %d connections to the topic of PX4'+CEND, self.pub.get_num_connections())# Modify
            if self.autopilot == "dji":
                rospy.loginfo(CBLUE2 +'There are %d connections to the topic of DJI'+CEND, self.pub.get_num_connections())
            if self.pub.get_num_connections() > 1:
                return 'gcs_connection'
            rospy.sleep(0.2)
        return 'shutdown'
