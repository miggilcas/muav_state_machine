#!/usr/bin/env python

import rospy
import smach
#from aerialcore_common.srv import ConfigMission, ConfigMissionResponse
from mavros_msgs.msg import State
from std_msgs.msg import Bool,String
from PrintColours import *


class MissionUpload(smach.State):
    def __init__(self,autopilot,uav_id):#modify
        smach.State.__init__(
            self, outcomes=['mission_commanded', 'shutdown'])
        self.autopilot = autopilot
        self.uav_id = uav_id
        

    def execute(self, userdata):
        rospy.loginfo('[MissionUpload] - MissionUpload state')
        airframe_pub = rospy.Publisher("/uav_{}_sm/com/airframe_type".format(self.uav_id), String, queue_size=10)
        
        mission_state_pub = rospy.Publisher("/uav_{}_sm/com/mission_state".format(self.uav_id), String, queue_size=10)
        if self.autopilot == "px4":
            airframe = self.autopilot + "/vtol"
        if self.autopilot == "dji":
            airframe = self.autopilot + "/M210"

        # transition to gcs_connection state
        while not rospy.is_shutdown():
            #publishers publishing
            airframe_pub.publish(airframe)
            mission_state_pub.publish("Mission Upload")
            
            if self.autopilot == "px4":    
                state_msg = rospy.wait_for_message("/uav_{}/mavros/state".format(self.uav_id), State)  # Modify
                if state_msg.mode == "AUTO.MISSION":
                    rospy.loginfo(CBLUE+"Vehicle with PX4 in {}, Mission commanded".format(state_msg.mode)+CEND)
                    return 'mission_commanded'
                else:
                    rospy.loginfo(CBLUE+"Vehicle with PX4 in {}".format(state_msg.mode)+CEND)
                    rospy.sleep(5)
            elif self.autopilot == "dji":
                rospy.loginfo(CBLUE+"Vehicle with DJI"+CEND)
                cmd_msg = rospy.wait_for_message("/uav_{}/dji_sm/command_mission".format(self.uav_id), Bool)
                if cmd_msg.data == True:
                    rospy.loginfo(CBLUE+"Mission commanded to the DJI "+CEND)
                    return 'mission_commanded'
            
        
            
        return 'shutdown'
