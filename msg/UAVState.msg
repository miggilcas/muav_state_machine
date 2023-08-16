string airframe_type # airframe type, please follow the format: autopilot / airframetype. Ex: px4/vtol, dji/M210
string mission_state 
uint8 wp_reached
uint8 uav_state  
uint8 landed_state 

# uav state for px4:
# uint8 VTOL_STATE_UNDEFINED = 0
# uint8 VTOL_STATE_TRANSITION_TO_FW = 1
# uint8 VTOL_STATE_TRANSITION_TO_MC = 2
# uint8 VTOL_STATE_MC = 3
# uint8 VTOL_STATE_FW = 4

# landed state for px4: 
# uint8 LANDED_STATE_UNDEFINED = 0
# uint8 LANDED_STATE_ON_GROUND = 1
# uint8 LANDED_STATE_IN_AIR = 2
# uint8 LANDED_STATE_TAKEOFF = 3
# uint8 LANDED_STATE_LANDING = 4
# landed state for dji:
# 0: STOPED
# 1: ON_GROUND
# 2: IN_AIR
# mavros_msgs/ExtendedState extended_state