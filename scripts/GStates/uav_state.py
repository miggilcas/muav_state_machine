#!/usr/bin/env python

import rospy
import smach
from PrintColours import *

from std_msgs.msg import String,UInt8
from mavros_msgs.msg import ExtendedState
# import custom message:
from muav_state_machine.msg import UAVState

# global variables to catch the data from the agent state machine node
autopilot = String() 
mission_state = String()
wp_reached = UInt8()
extended_state = ExtendedState()

#callback functions
def autopilot_cb(msg):
    autopilot = msg

def mission_state_cb(msg):
    mission_state = msg

def wp_reached_cb(msg):
    wp_reached = msg

def estate_cb(msg):
    extended_state = msg

class UavState(smach.State):
    def __init__(self,uav_id):#modify, common_data
        smach.State.__init__(
            self, outcomes=['mission_finished', 'shutdown'])
        self.uav_id = uav_id
        

    def execute(self, ud):
        UAVState_pub = rospy.Publisher("/muav_sm/uav_{}/UAVState".format(self.uav_id), UAVState, queue_size=10)
        rospy.loginfo('[UavState] - UAV{} state'.format(self.uav_id))
        rate = rospy.Rate(10) # 10hz
        UAVState_msg = UAVState()
        #subscribers initialization
        autopilot_sub = rospy.Subscriber("/uav_{}_sm/com/autopilot".format(self.uav_id), String, autopilot_cb)
        mission_state_sub = rospy.Subscriber("/uav_{}_sm/com/mission_state".format(self.uav_id), String, mission_state_cb)
        wp_reached_sub = rospy.Subscriber("/uav_{}_sm/com/wp_reached".format(self.uav_id), UInt8, wp_reached_cb)
        extended_state_sub = rospy.Subscriber("/uav_{}_sm/com/extended_state".format(self.uav_id), ExtendedState, estate_cb)
        
        # UAVState_msg initialization
        UAVState_msg.autopilot = "px4"
        UAVState_msg.mission_state = "idle"
        UAVState_msg.wp_reached = 0
        UAVState_msg.extended_state.vtol_state = 0
        UAVState_msg.extended_state.landed_state = 0
        # transition to X state
        while not rospy.is_shutdown():           
            # TBD: error detection if not namespaces with the name of the uav_id 
            

            rospy.loginfo('[UavState] - UAV{} state: autopilot: {}, mission_state: {}, wp_reached: {}, extended_state: {}'.format(self.uav_id,autopilot,mission_state,wp_reached,extended_state))
            
            #fill the UAVState_msg custom message
            UAVState_msg.autopilot = autopilot #parameter
            UAVState_msg.mission_state = mission_state#published by the agent state machine
            if UAVState_msg.autopilot=="px4" and mission_state=="mission_running":
                UAVState_msg.wp_reached = wp_reached #published by the agent state machine
                UAVState_msg.extended_state = extended_state #published by the agent state machine
            else:
                UAVState_msg.wp_reached = 0
                UAVState_msg.extended_state.vtol_state = 0
                UAVState_msg.extended_state.landed_state = 0
            

            #publish the UAVState_msg custom message
            UAVState_pub.publish(UAVState_msg) 
            

            #finish this state:
            # if UAVState_msg.mission_state == "idle":
            #     return 'mission_finished'
            
            rate.sleep()
        return 'shutdown'
