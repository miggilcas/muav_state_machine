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
from AgentStates.rth import ReturnToHome


from mavros_msgs.msg import State
from mavros_msgs.srv import WaypointClear,WaypointClearRequest

from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from PrintColours import *



class MUAVStateMachine:
    def __init__(self):
        rospy.init_node('agent_sm_node')
        logger = logging.getLogger("rosout")
        logger.setLevel(logging.INFO)
        #Getting parameters
        uav_id_search = rospy.search_param('uav_id')
        autopilot_search = rospy.search_param('autopilot')

        uav_id = rospy.get_param(uav_id_search)
        autopilot = rospy.get_param(autopilot_search)
        
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['shutdown'])
        # we define the user data
        self.sm.userdata.wp_reached = 0
        self.sm.userdata.wp_list = []
        self.sm.userdata.ext_state = 0
        

        # In order to know how many connections are active we create a publisher
        if autopilot=="px4":
            #pub = rospy.Publisher("/uav_{}/mavros/battery".format(uav_id), BatteryState, queue_size=10)
            pub = rospy.Publisher("/uav_{}/mavros/global_position/global".format(uav_id), NavSatFix, queue_size=10)
            
        elif autopilot=="dji":
            pub = rospy.Publisher("/uav_{}/dji_osdk_ros/battery_state".format(uav_id), BatteryState, queue_size=10)
        else:
            pass
            
        # Open the container
        with self.sm:
            # # Add states to the container
            smach.StateMachine.add('IDLE', Idle(pub,autopilot,uav_id),
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
                                 'rth' : 'RTH',
                                 'shutdown'  : 'shutdown'},
                    remapping={'mission_wp_r' : 'wp_reached',
                               'mission_wp_l' : 'wp_list',
                               'mission_ext_state':'ext_state',
                               'mission_wp_r_' : 'wp_reached',
                               'mission_wp_l_' : 'wp_list',
                               'mission_ext_state_':'ext_state'})
            
            ## Concurrence State machine that will be monitoring the mission state

            # Tests
            ## Will be the CANCEL State for PX4 or mavros aircrafts
            smach.StateMachine.add('CLEARMISSION', 
                                   smach_ros.ServiceState('/uav_{}/mavros/mission/clear'.format(uav_id), WaypointClear,
                                request = WaypointClearRequest()),
                    transitions={'succeeded':'IDLE',
                                 'aborted':'IDLE',
                                 'preempted':'IDLE'})
            
            # Emergencies

            ## RTH State
            smach.StateMachine.add('RTH', ReturnToHome(autopilot,uav_id),
                    transitions={'idle' : 'IDLE',
                                 #'resume': 'RESUME',
                                 'clear_mission' : 'CLEARMISSION',
                                 'shutdown'  : 'shutdown'},
                    remapping={'rth_wp_r' : 'wp_reached',
                               'rth_wp_l' : 'wp_list',
                               'rth_ext_state':'ext_state',
                               'rth_wp_r_' : 'wp_reached',
                               'rth_wp_l_' : 'wp_list',
                               'rth_ext_state_':'ext_state'})
            ## STOP State

            ## RESUME State
           
            

            
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
