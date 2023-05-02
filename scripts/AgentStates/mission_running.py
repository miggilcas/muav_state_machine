#!/usr/bin/env python

import rospy
import smach
from mavros_msgs.msg import State, WaypointReached, WaypointList, ExtendedState
from std_msgs.msg import UInt8
from PrintColours import *

# callback functions to monitor the topics
def wp_reached_cb(msg):
    #rospy.loginfo(CGREEN+"waypoint reached: {}".format(msg.wp_seq)+CEND)
    return msg
def wp_list_cb(msg):
    #rospy.loginfo(CYELLOW+"current wp:{} waypoint list: {}".format(msg.current_seq,msg.waypoints)+CEND)
    return msg

def extended_state_cb(msg):
    #rospy.loginfo(CVIOLET+"vtol state: {}, landed state: {}".format(msg.vtol_state,msg.landed_state)+CEND)
    return msg



class MissionRunning(smach.State):
    def __init__(self,autopilot,uav_id):
        smach.State.__init__(
            self, outcomes=['idle','clear_mission','shutdown'],output_keys=['waypoint_reached','waypoints','extended_state'])
        self.autopilot = autopilot
        self.uav_id = uav_id

    def execute(self, ud):
        rospy.loginfo('[MissionRunning] - Mission_running state')
        rospy.set_param("/uav_{}_sm/autopilot".format(self.uav_id),"MissionRunning")
        # Monitor subscribers
        wp_reached_sub = rospy.Subscriber("/uav_{}/mavros/mission/reached".format(self.uav_id), WaypointReached, wp_reached_cb)
        wp_list_sub = rospy.Subscriber("/uav_{}/mavros/mission/waypoints".format(self.uav_id), WaypointList, wp_list_cb)
        extended_state_sub = rospy.Subscriber("/uav_{}/mavros/extended_state".format(self.uav_id), ExtendedState, extended_state_cb)

        #Monitor publishers
        wp_reached_pub = rospy.Publisher("/uav_{}/mavros/mission/reached".format(self.uav_id), WaypointReached, queue_size=10)
        extended_state_pub = rospy.Publisher("/uav_{}/mavros/extended_state".format(self.uav_id), ExtendedState, queue_size=10)
        # Transition to finish the mission
        while not rospy.is_shutdown():
            if self.autopilot == "px4":
                
                # user data
                ud.waypoint_reached = wp_reached_cb#rospy.wait_for_message("/uav_{}/mavros/mission/reached".format(self.uav_id), WaypointReached)
                ud.waypoints = wp_list_cb#rospy.wait_for_message("/uav_{}/mavros/mission/waypoints".format(self.uav_id), WaypointList)
                # ud.vtol_state = extended_state_cb.vtol_state
                # ud.landed_state = extended_state_cb.landed_state
                ud.extended_state = extended_state_cb
                state_msg = rospy.wait_for_message("/uav_{}/mavros/state".format(self.uav_id), State)
                
                # publishing for monitor from GS
                wp_reached_pub.publish(ud.waypoint_reached)
                extended_state_pub.publish(ud.extended_state)

                #Transitions:
                if state_msg.armed:
                    #rospy.loginfo(CBLUE+"Vehicle armed yet"+CEND)
                    pass
                else:
                    rospy.loginfo(CBLUE+"Vehicle with PX4 disarmed, mission finished"+CEND)
                    
                    # transition to a service call to clear the mission, it could be another state, when everything is done in the mission 
                    return 'clear_mission'
                    #return 'idle'
            elif self.autopilot == "dji":
                state_msg = rospy.wait_for_message("/uav_{}/dji_osdk_ros/flight_status".format(self.uav_id), UInt8)
                if state_msg.data == 0: # Landed
                    rospy.loginfo(CBLUE+"Vehicle with DJI landing, MISSION FINISHED"+CEND)
                    return 'idle'
            # if (True):# Modify
            #     rospy.sleep(15)
            #     return 'idle'
            
            rospy.sleep(2)
           # TODO: Add a service call to stop or pause the mission in case of emergency
           
        return 'shutdown'
