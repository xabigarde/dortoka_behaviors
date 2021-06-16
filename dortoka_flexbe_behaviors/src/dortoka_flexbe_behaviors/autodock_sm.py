#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dortoka_flexbe_states.autodock_state import AutodockState
from dortoka_flexbe_states.battery_monitor_state import BatteryMonitor
from dortoka_flexbe_states.go_forward_state import GoForwardState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 15 2021
@author: xabi
'''
class AutoDockSM(Behavior):
    '''
    Automatically docks.
Requires the following launchers to be run in separate terminals:
$ roslaunch turtlebot_bringup minimal.launch
$ roslaunch kobuki_auto_docking standalone.launch
    '''


    def __init__(self):
        super(AutoDockSM, self).__init__()
        self.name = 'AutoDock'

        # parameters of this behavior
        self.add_parameter('drive_speed', -0.2)
        self.add_parameter('travel_distance', 0.3)
        self.add_parameter('obstacle_distance', 0.3)
        self.add_parameter('battery_threshold', 164)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:619 y:291, x:385 y:288
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:88 y:92
            OperatableStateMachine.add('BatteryMonitor',
                                        BatteryMonitor(battery_threshold=self.battery_threshold),
                                        transitions={'failed': 'failed', 'low_battery': 'Drive Fwd'},
                                        autonomy={'failed': Autonomy.Off, 'low_battery': Autonomy.Off})

            # x:566 y:70
            OperatableStateMachine.add('Dock',
                                        AutodockState(),
                                        transitions={'done': 'finished', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:341 y:81
            OperatableStateMachine.add('Drive Fwd',
                                        GoForwardState(speed=self.drive_speed, travel_dist=self.travel_distance, obstacle_dist=self.obstacle_distance),
                                        transitions={'failed': 'failed', 'done': 'Dock'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
