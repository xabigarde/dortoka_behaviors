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
from dortoka_flexbe_states.charging_state import Charging
from dortoka_flexbe_states.find_dock_state import FindDockState
from dortoka_flexbe_states.go_forward_state import GoForwardState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 18 2021
@author: xabi
'''
class BatteryMonitorSM(Behavior):
    '''
    Monitors the battery
    '''


    def __init__(self):
        super(BatteryMonitorSM, self).__init__()
        self.name = 'BatteryMonitor'

        # parameters of this behavior
        self.add_parameter('battery_threshold', 163)
        self.add_parameter('wait_time', 10)
        self.add_parameter('autodock_timeout', 15)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:716 y:94, x:262 y:269
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

        # x:212 y:582, x:671 y:214
        _sm_gotodockprioritycontainer_0 = PriorityContainer(outcomes=['finished', 'failed'])

        with _sm_gotodockprioritycontainer_0:
            # x:59 y:32
            OperatableStateMachine.add('FindDock',
                                        FindDockState(),
                                        transitions={'failed': 'failed', 'charger_found': 'Autodock'},
                                        autonomy={'failed': Autonomy.Off, 'charger_found': Autonomy.Low})

            # x:5 y:239
            OperatableStateMachine.add('BackOff',
                                        GoForwardState(speed=-0.1, travel_dist=0.1, obstacle_dist=0.3),
                                        transitions={'failed': 'failed', 'done': 'FindDock'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Low})

            # x:177 y:350
            OperatableStateMachine.add('Charging',
                                        Charging(),
                                        transitions={'failed': 'failed', 'fully_charged': 'Undock', 'charging': 'Charging', 'discharging': 'BackOff'},
                                        autonomy={'failed': Autonomy.Off, 'fully_charged': Autonomy.Low, 'charging': Autonomy.Off, 'discharging': Autonomy.Low})

            # x:175 y:464
            OperatableStateMachine.add('Undock',
                                        GoForwardState(speed=-0.1, travel_dist=0.5, obstacle_dist=0.3),
                                        transitions={'failed': 'failed', 'done': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Low})

            # x:305 y:245
            OperatableStateMachine.add('WaitTillFullyCharged',
                                        WaitState(wait_time=self.wait_time),
                                        transitions={'done': 'WaitTillFullyCharged'},
                                        autonomy={'done': Autonomy.Off})

            # x:160 y:165
            OperatableStateMachine.add('Autodock',
                                        AutodockState(timeout=self.autodock_timeout),
                                        transitions={'done': 'Charging', 'failed': 'failed', 'timed_out': 'FindDock'},
                                        autonomy={'done': Autonomy.Low, 'failed': Autonomy.Off, 'timed_out': Autonomy.Off})



        with _state_machine:
            # x:113 y:91
            OperatableStateMachine.add('BatteryMonitor',
                                        BatteryMonitor(battery_threshold=self.battery_threshold),
                                        transitions={'failed': 'failed', 'low_battery': 'GoToDockPriorityContainer', 'already_docked': 'GoToDockPriorityContainer'},
                                        autonomy={'failed': Autonomy.Off, 'low_battery': Autonomy.Low, 'already_docked': Autonomy.Low})

            # x:372 y:86
            OperatableStateMachine.add('GoToDockPriorityContainer',
                                        _sm_gotodockprioritycontainer_0,
                                        transitions={'finished': 'BatteryMonitor', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
