#!/usr/bin/env python3

import rospy
import logging
import smach
import smach_ros

from std_msgs.msg import String, UInt8

from AgentStates.idle      import Idle
from AgentStates.gcs_connection import GCSConnection
from AgentStates.mission_upload import MissionUpload
from AgentStates.mission_commanded import MissionCommanded
from AgentStates.mission_running import MissionRunning


from mavros_msgs.msg import State
from mavros_msgs.srv import WaypointClear,WaypointClearRequest

from sensor_msgs.msg import BatteryState
from PrintColours import *



class MUAVStateMachine:
    def __init__(self):
        rospy.init_node('agent_sm_node')
        logger = logging.getLogger("rosout")
        logger.setLevel(logging.INFO)
        #Getting parameters
        uav_id = rospy.get_param("/agent_sm_node/uav_id")
        autopilot = rospy.get_param("/agent_sm_node/autopilot")

        #Setting the parameters for eassy access to GS_sm_node
        rospy.set_param("/uav_{}_sm/autopilot".format(uav_id),autopilot)
        rospy.set_param("/uav_{}_sm/mission_state".format(uav_id),"idle")

        #MonitorMavrosState_topic_cb = lambda ud, msg: msg.armed  # callback function to monitor the topic '/mavros/state'
        # def MonitorMavrosState_topic_cb(ud, msg):
        #     ud.armed = msg.armed
        #     rospy.sleep(1.0)  # actualizar cada 1 segundo

        # MonitorMavrosState_cb = smach_ros.MonitorState('/uav_2/mavros/state', State, MonitorMavrosState_topic_cb, output_keys=['armed'])  # creating a MonitorState object

        #sm_data = AmuledStateMachineData()
        #states = sm_data.states
        
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['shutdown'])
        # def state_callback(state_msg):
        #     if state_msg.armed:
        #         rospy.loginfo("Vehicle armed")
        #     else:
        #         rospy.loginfo("Vehicle disarmed")
        

        # In order to know how many connections are active we create a publisher
        if autopilot=="px4":
            pub = rospy.Publisher("/uav_{}/mavros/battery".format(uav_id), BatteryState, queue_size=10)
        elif autopilot=="dji":
            pub = rospy.Publisher("/uav_{}/dji_osdk_ros/battery_state".format(uav_id), BatteryState, queue_size=10)
        else:
            pass
            #pub = rospy.Publisher("/uav_{}/mavros/battery".format(uav_id), BatteryState, queue_size=10)    
        # create a subscriber to the mavros/state topic
        #sub = rospy.Subscriber("/uav_2/mavros/state", State, state_callback)
        # Open the container
        with self.sm:
            # # Add states to the container
            smach.StateMachine.add('IDLE', Idle(pub,autopilot),
                    transitions={'gcs_connection' : 'GCSCONNECTION',
                                 'shutdown'  : 'shutdown'})
                     
            smach.StateMachine.add('GCSCONNECTION', GCSConnection(autopilot,uav_id),# pasar el id y el autopiloto
                    transitions={'mission_upload' : 'MISSIONUPLOAD',
                                 'shutdown'  : 'shutdown'})
            smach.StateMachine.add('MISSIONUPLOAD', MissionUpload(autopilot,uav_id),
                    transitions={'mission_commanded' : 'MISSIONCOMMANDED',
                                 'shutdown'  : 'shutdown'})
            smach.StateMachine.add('MISSIONCOMMANDED', MissionCommanded(autopilot,uav_id),
                    transitions={'mission_running' : 'MISSIONRUNNING',
                                 'shutdown'  : 'shutdown'})
            smach.StateMachine.add('MISSIONRUNNING', MissionRunning(autopilot,uav_id),
                    transitions={'idle' : 'IDLE',
                                 'clear_mission' : 'CLEARMISSION',
                                 'shutdown'  : 'shutdown'})
            
            ## Concurrence State machine that will be monitoring the mission state

            # pruebas
            smach.StateMachine.add('CLEARMISSION', 
                                   smach_ros.ServiceState('/uav_{}/mavros/mission/clear'.format(uav_id), WaypointClear,
                                request = WaypointClearRequest()),
                    transitions={'succeeded':'IDLE',
                                 'aborted':'IDLE',
                                 'preempted':'IDLE'})
           
            

            
        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer(
            'muav_state_machine_server', self.sm, '/muav_state_machine_sm')
        self.sis.start()


    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sis.stop()

if __name__ == '__main__':
    # Create the top level SMACH state machine
    with MUAVStateMachine() as muav_state_machine_sm:
        muav_state_machine_sm.sm.execute()
