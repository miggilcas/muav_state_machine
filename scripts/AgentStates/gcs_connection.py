#!/usr/bin/env python

import rospy
import smach
from mavros_msgs.msg import WaypointList
from std_msgs.msg import Bool
from PrintColours import *
#from aerialcore_common.srv import ConfigMission, ConfigMissionResponse

# 
# def mission_callback(req):
#     rospy.loginfo(" /mission/new service was called")
#     return ConfigMissionResponse(success=True)


class GCSConnection(smach.State):
    def __init__(self,autopilot,uav_id):
        smach.State.__init__(
            self, outcomes=['mission_upload', 'shutdown'])
        #rospy.Service('uav_2/mission/new', ConfigMission, mission_callback)
        self.autopilot = autopilot
        self.uav_id = uav_id

    def execute(self, ud):
        rospy.loginfo('[GCSconnection] - GCSconnection state')
        rospy.set_param("/uav_{}_sm/autopilot".format(self.uav_id),"GCSconnection")
        
        # transition to gcs_connection state
        while not rospy.is_shutdown():
            if self.autopilot == "px4":
                waypointList_msg = rospy.wait_for_message("/uav_{}/mavros/mission/waypoints".format(self.uav_id), WaypointList)# modify
                rospy.loginfo(CBLUE+"There are %d waypoints in the mission"+CEND, len(waypointList_msg.waypoints))
                if len(waypointList_msg.waypoints) > 0:
                    rospy.loginfo("Vehicle with PX4 with MISSION LOADED has %d waypoints", len(waypointList_msg.waypoints))
                    return 'mission_upload'
                else:
                    rospy.loginfo("Vehicle with PX4 have no mission loaded")
                    rospy.sleep(1)
            elif self.autopilot == "dji":
                rospy.loginfo(CBLUE+"Vehicle with DJI"+CEND)
                cmd_msg = rospy.wait_for_message("/uav_{}/dji_sm/upload_mission".format(self.uav_id), Bool)
                if cmd_msg.data == True:
                    rospy.loginfo(CBLUE+"Vehicle with DJI has mission loaded"+CEND)
                    return 'mission_upload'
            # if (True):# Modify
            #     rospy.sleep(2)
            #     return 'mission_upload'
            
        return 'shutdown'
