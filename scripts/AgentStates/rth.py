#!/usr/bin/env python

import rospy
import smach
from PrintColours import *
from std_msgs.msg import String
from mavros_msgs.msg import State

#services
#from dji_osdk_ros.srv import *
from mavros_msgs.srv import *
from std_srvs.srv import SetBool




class ReturnToHome(smach.State):
    def __init__(self,autopilot,uav_id):#modify, common_data
        smach.State.__init__(
            self, outcomes=['idle','clear_mission', 'shutdown'],input_keys=['rth_wp_r','rth_wp_l','rth_wp_ext_state'],#, 'resume'
                                    output_keys=['rth_wp_r_','rth_wp_l_','rth_wp_ext_state_'])
        self.autopilot = autopilot
        self.uav_id = uav_id

    def execute(self, userdata):
        rospy.loginfo('[RTH] - Return To Home state')
        airframe_pub = rospy.Publisher("/uav_{}_sm/com/airframe_type".format(self.uav_id), String, queue_size=10)
        
        mission_state_pub = rospy.Publisher("/uav_{}_sm/com/mission_state".format(self.uav_id), String, queue_size=10)
        if self.autopilot == "px4":
            airframe = self.autopilot + "/vtol"
            

        if self.autopilot == "dji":
            airframe = self.autopilot + "/M210"
        
        while not rospy.is_shutdown():      
            airframe_pub.publish(airframe)
            mission_state_pub.publish("RTH")

            if self.autopilot == "px4":
                state_msg = rospy.wait_for_message("/uav_{}/mavros/state".format(self.uav_id), State)  # Modify
                if state_msg.mode == "AUTO.MISSION":
                    rospy.loginfo(CBLUE2 +'Calling the PX4 Return To Home Service'+CEND)
                    self.rth_px4_srv()
                if state_msg.mode == "AUTO.RTL":
                    #TBD: Calculate the distance between points 
                    rospy.loginfo(CBLUE2 +'Returning Home, Distance:  m'+CEND)
                    state_msg = rospy.wait_for_message("/uav_{}/mavros/state".format(self.uav_id), State)
                    if state_msg.armed :#and emergency==False
                        #rospy.loginfo(CBLUE+"Vehicle armed yet"+CEND)
                        pass
                    else:
                        rospy.loginfo(CBLUE+"PX4 Vehicle disarmed, mission finished"+CEND)
                        
                        # TBD: call the service to clear the mission and then do the transition to idle or resume depending on the emergency command
                        return 'clear_mission'

                
            
            if self.autopilot == "dji":
                rospy.loginfo(CBLUE2 +'Calling the DJI Return To Home Service'+CEND)
            
            
            rospy.sleep(0.1)
        return 'shutdown'
    
    # Methods For returning to home
    def rth_px4_srv(self):
        rospy.wait_for_service('/uav_{}/mission/start_stop'.format(self.uav_id)) #test if it is needed the namespace or not
        try:
            rth_cmd = rospy.ServiceProxy('/uav_{}/mission/start_stop'.format(self.uav_id),SetBool)
            res = rth_cmd(data = False)
            return res.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)