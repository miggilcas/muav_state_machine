#!/usr/bin/env python

import rospy
import smach
#from aerialcore_common.srv import ConfigMission, ConfigMissionResponse
from mavros_msgs.msg import State
from std_msgs.msg import Bool,String
from PrintColours import *

# def mission_callback(req):
#     rospy.loginfo(" /mission/new service was called")
#     return ConfigMissionResponse(success=True)

class MissionUpload(smach.State):
    def __init__(self,autopilot,uav_id):#modify
        smach.State.__init__(
            self, outcomes=['mission_commanded', 'shutdown'])
        self.autopilot = autopilot
        self.uav_id = uav_id
        #rospy.Service('uav_2/mission/new', ConfigMission, mission_callback)
        

    def execute(self, ud):
        rospy.loginfo('[MissionUpload] - MissionUpload state')
        # rospy.set_param("/uav_{}_sm/autopilot".format(self.uav_id),"MissionUpload") changed
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
                    rospy.loginfo(CBLUE+"Vehicle with PX4 in {}, Mission commanded".format(state_msg)+CEND)
                    return 'mission_commanded'
                else:
                    rospy.loginfo(CBLUE+"Vehicle with PX4 in {}".format(state_msg)+CEND)
                    rospy.sleep(1)
            elif self.autopilot == "dji":
                rospy.loginfo(CBLUE+"Vehicle with DJI"+CEND)
                cmd_msg = rospy.wait_for_message("/uav_{}/dji_sm/command_mission".format(self.uav_id), Bool)
                if cmd_msg.data == True:
                    rospy.loginfo(CBLUE+"Mission commanded to the DJI "+CEND)
                    return 'mission_commanded'
            
            # if (True):# Modify
            #     rospy.sleep(3)
            #     return 'mission_commanded'
            
            
        return 'shutdown'
