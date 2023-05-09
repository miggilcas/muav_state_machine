#!/usr/bin/env python

import rospy
import smach
from PrintColours import *

from std_msgs.msg import String,UInt8
from mavros_msgs.msg import ExtendedState
# import custom message:
from muav_state_machine.msg import UAVState

# global variables to catch the data from the agent state machine node
airframe_type = String()
mission_state = String()
wp_reached = UInt8()
extended_state = ExtendedState()
uav_state = UInt8()
landed_state = UInt8()
flight_status_dji = UInt8()

#callback functions
def airframe_type_cb(msg):
    global airframe_type 
    airframe_type = msg

def mission_state_cb(msg):
    global mission_state
    mission_state = msg

def wp_reached_cb(msg):
    global wp_reached
    wp_reached = msg

def estate_cb(msg):
    global extended_state
    extended_state = msg

def flight_status_dji_cb(msg):
    global flight_status_dji
    flight_status_dji = msg
    
class UavState(smach.State):
    def __init__(self,uav_id):#modify, common_data
        smach.State.__init__(
            self, outcomes=['mission_finished', 'shutdown'])
        self.uav_id = uav_id
        

    def execute(self, ud):
        UAVState_pub = rospy.Publisher("/muav_sm/uav_{}/uavstate".format(self.uav_id), UAVState, queue_size=10)
        rospy.loginfo('[UavState] - UAV{} state'.format(self.uav_id))
        rate = rospy.Rate(20) # 20hz
        UAVState_msg = UAVState()
        #subscribers initialization
        autopilot_sub = rospy.Subscriber("/uav_{}_sm/com/airframe_type".format(self.uav_id), String, airframe_type_cb)
        mission_state_sub = rospy.Subscriber("/uav_{}_sm/com/mission_state".format(self.uav_id), String, mission_state_cb)
        wp_reached_sub = rospy.Subscriber("/uav_{}_sm/com/wp_reached".format(self.uav_id), UInt8, wp_reached_cb)
        if airframe_type=="px4/vtol":
            extended_state_sub = rospy.Subscriber("/uav_{}_sm/com/extended_state".format(self.uav_id), ExtendedState, estate_cb)
        # subscribers for dji data
        if airframe_type=="dji/M210":
            flight_status_dji_sub = rospy.Subscriber("/uav_{}_sm/com/flight_status_dji".format(self.uav_id), UInt8, flight_status_dji_cb)
        # UAVState_msg initialization
        UAVState_msg.airframe_type = "px4/vtol"
        UAVState_msg.mission_state = "idle"
        UAVState_msg.wp_reached = 0
        UAVState_msg.uav_state = 0
        UAVState_msg.landed_state = 0
        # transition to X state
        while not rospy.is_shutdown():           
            # TBD: error detection if not namespaces with the name of the uav_id 
            

            #rospy.loginfo('[UavState] - UAV{} state: airframetype: {}, mission_state: {}, wp_reached: {}, extended_state: {}'.format(self.uav_id,autopilot,mission_state,wp_reached,extended_state))
            
            #fill the UAVState_msg custom message
            UAVState_msg.airframe_type = airframe_type #parameter
            UAVState_msg.mission_state = mission_state#published by the agent state machine
            if airframe_type=="px4/vtol" and mission_state=="mission_running":
                UAVState_msg.wp_reached = wp_reached #published by the agent state machine
                UAVState_msg.uav_state = extended_state.vtol_state #published by the agent state machine
                UAVState_msg.landed_state = extended_state.landed_state
            else:
                UAVState_msg.wp_reached = 0 #TBD: create a function to do it in the DJI
                UAVState_msg.uav_state = 0 
                UAVState_msg.landed_state = 0 # modified with dji data
                if airframe_type=="dji/M210" and mission_state=="mission_running":
                    UAVState_msg.landed_state = flight_status_dji #published by the agent state machine
            

            #publish the UAVState_msg custom message
            UAVState_pub.publish(UAVState_msg) 
            

            #finish this state:
            # if UAVState_msg.mission_state == "idle":
            #     return 'mission_finished'
            
            rate.sleep()
        return 'shutdown'
