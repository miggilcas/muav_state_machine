#!/usr/bin/env python
'''
    This node will be in charge of analyzing the general aspects of the complete mission involving all UAVs.
    Of course it will be running in the Ground Station (GS) computer.

'''

import rospy
import logging
import smach
import smach_ros
from GStates.idle      import Idle
from GStates.uav_state      import UavState

from GStates.mission_finished      import MissionFinished

# imports for GS
import rosnode # rosnode.get_node_names() for getting all the active nodes instead of doing it manually from the terminal

# debug variables
n_uavs = 2

class GSStateMachine:
    def __init__(self):
        rospy.init_node('GS_sm_node')
        logger = logging.getLogger("rosout")
        logger.setLevel(logging.INFO)
        
        
        # def masters_result_cb(userdata, status, result):
        #     if status == DiscoverMastersRequest.SUCCEEDED:
        #         userdata.gripper_output = result.num_iterations
        #         return 'my_outcome'
        
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['shutdown'])
        
        
        
        
            
        # Open the container
        with self.sm:
            # smach.StateMachine.add('IDLE', 
            #                        smach_ros.ServiceState('"/uav_{}/mavros/mission/clear'.format(uav_id), WaypointClear,
            #                     request = DiscoverMastersRequest()),
            #                     result_cb=masters_result_cb,
            #         transitions={'succeeded':'IDLE'})
            # # Add states to the container
            smach.StateMachine.add('IDLE', Idle(),
                    transitions={'UAVs_monitoring':'CON',
                                'shutdown'  : 'shutdown'})
            
            # Create a sub state machine with concurrence states for each UAV
            self.sm_uavs = smach.Concurrence(outcomes=['outcome1','shutdown'],
                                             default_outcome='shutdown',
                                             child_termination_cb = self.child_term_cb,
                                             outcome_cb = self.out_cb)

            # Open the container
            with self.sm_uavs:
                for i in range (1,n_uavs):
                    smach.Concurrence.add('UAV{}_STATE'.format(i), UavState(i))
            
            smach.StateMachine.add('CON', self.sm_uavs,
                               transitions={'outcome1':'MISSIONFINISHED',
                                            'shutdown':'shutdown'})
            smach.StateMachine.add('MISSIONFINISHED', MissionFinished(),
                    transitions={'idle' : 'IDLE',# 'clear_mission' : 'CLEARMISSION',
                                 
                                 'shutdown'  : 'shutdown'})
            
        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer(
            'GS_sm_server', self.sm, '/GS_sm')
        self.sis.start()

    def child_term_cb(self, outcome_map):# this function and the following could be modified to be more complex
        # for example it could be used to finished the Concurrent subsate machine if one enter in emergency mode
        return False
    
    def out_cb(self, outcome_map):
        return 'outcome1'
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sis.stop()

if __name__ == '__main__':
    # Create the top level SMACH state machine
    with GSStateMachine() as GS_sm:
        GS_sm.sm.execute()
