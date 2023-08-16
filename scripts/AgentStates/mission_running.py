#!/usr/bin/env python

import rospy
import smach
from mavros_msgs.msg import State, WaypointReached, WaypointList, ExtendedState
from dji_osdk_ros.msg import MissionWaypointTask
from std_msgs.msg import UInt8,String
from PrintColours import *

#services
from dji_osdk_ros.srv import *
from mavros_msgs.srv import *
from muav_state_machine.srv import EmergencyCmd,EmergencyCmdResponse

def rth__srv():
    rospy.wait_for_service('mission/start_stop') #test if it is needed the namespace or not
    try:
        rth_cmd = rospy.ServiceProxy('mission/start_stop')
        res = rth_cmd()
        return res.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def handle_emergencies(req):
    global emergency
    emergency = req.emergency
    rospy.loginfo("Emergency command received: {}".format(emergency))

    return EmergencyCmdResponse

# global variables for communication


# callback functions to monitor the topics
def wp_reached_cb(msg):
    global wp_reached
    wp_reached = msg

def wp_list_cb(msg):
    global wp_list
    wp_list = msg

def wp_list_dji_cb(msg):
    global wp_list
    wp_list = msg

def extended_state_cb(msg):
    global extended_state
    extended_state = msg



class MissionRunning(smach.State):
    def __init__(self,autopilot,uav_id):
        smach.State.__init__(
            self, outcomes=['idle','clear_mission','shutdown'],output_keys=['waypoint_reached','waypoints','extended_state'])
        self.autopilot = autopilot
        self.uav_id = uav_id

    def execute(self, ud):
        rospy.loginfo('[MissionRunning] - Mission_running state')
        
        #Emergency cmds service
        emergency_srv = rospy.Service('/uav_{}/emer_cmd'.format(self.uav_id), EmergencyCmd, handle_emergencies)

        #Monitor publishers
        airframe_pub = rospy.Publisher("/uav_{}_sm/com/airframe_type".format(self.uav_id), String, queue_size=10)
        mission_state_pub = rospy.Publisher("/uav_{}_sm/com/mission_state".format(self.uav_id), String, queue_size=10)
        
        if self.autopilot == "px4":
            airframe = self.autopilot + "/vtol"
        if self.autopilot == "dji":
            airframe = self.autopilot + "/M210"
        
        wp_reached_pub = rospy.Publisher("/uav_{}_sm/com/wp_reached".format(self.uav_id), WaypointReached, queue_size=10)

        # Different services, publishers and subscribers depending on the autopilot
        if self.autopilot == "px4":
            extended_state_pub = rospy.Publisher("/uav_{}_sm/com/extended_state".format(self.uav_id), ExtendedState, queue_size=10)
            
            # Monitor subscribers
            wp_reached_sub = rospy.Subscriber("/uav_{}/mavros/mission/reached".format(self.uav_id), WaypointReached, wp_reached_cb)
            wp_list_sub = rospy.Subscriber("/uav_{}/mavros/mission/waypoints".format(self.uav_id), WaypointList, wp_list_cb)
            extended_state_sub = rospy.Subscriber("/uav_{}/mavros/extended_state".format(self.uav_id), ExtendedState, extended_state_cb)

            # Command services
            RTH_client = rospy.ServiceProxy('mission/start_stop')

        if self.autopilot == "dji":
            flight_status_dji_pub = rospy.Publisher("/uav_{}_sm/com/flight_status_dji".format(self.uav_id), UInt8, queue_size=10)
            # Monitor subscribers
            wp_reached_sub = rospy.Subscriber("/uav_{}/dji_sm/wp_reached".format(self.uav_id), UInt8, wp_reached_cb)
            wp_list_sub = rospy.Subscriber("/uav_{}/dji_sm/wp_list".format(self.uav_id), MissionWaypointTask, wp_list_dji_cb)
            
            # Command services
            RTH_client = rospy.ServiceProxy('uav_{}/dji_osdk_ros/'.format(self.uav_id))


        # Transition to finish the mission
        while not rospy.is_shutdown():
            airframe_pub.publish(airframe)
            mission_state_pub.publish("Mission running")

            



            if self.autopilot == "px4":
                
                # user data, useful for other states
                ud.waypoint_reached = wp_reached
                ud.waypoints = wp_list
                # ud.vtol_state = extended_state_cb.vtol_state
                # ud.landed_state = extended_state_cb.landed_state
                ud.extended_state = extended_state
                state_msg = rospy.wait_for_message("/uav_{}/mavros/state".format(self.uav_id), State)
                
                # publishing for monitor from GS
                wp_reached_pub.publish(ud.waypoint_reached)
                extended_state_pub.publish(ud.extended_state)

                #Transitions:  ##TBD: Control the emergencies
                if state_msg.armed and emergency==False:
                    #rospy.loginfo(CBLUE+"Vehicle armed yet"+CEND)
                    pass
                else:
                    rospy.loginfo(CBLUE+"PX4 Vehicle disarmed, mission finished"+CEND)
                    
                    # transition to a service call to clear the mission, it could be another state, when everything is done in the mission 
                    return 'clear_mission'
                    #return 'idle'
            elif self.autopilot == "dji":
                state_msg = rospy.wait_for_message("/uav_{}/dji_osdk_ros/flight_status".format(self.uav_id), UInt8)
                flight_status_dji_pub.publish(state_msg)
                if state_msg.data == 0 and emergency==False: # state.data = 0: Landed ##TBD: Control the emergencies
                    rospy.loginfo(CBLUE+"DJI Vehicle  landing, MISSION FINISHED"+CEND)
                    return 'idle'
            # if (True):# Modify
            #     rospy.sleep(15)
            #     return 'idle'
            
            rospy.sleep(2)
           # TODO: Add a service call to stop or pause the mission in case of emergency
           
        return 'shutdown'
