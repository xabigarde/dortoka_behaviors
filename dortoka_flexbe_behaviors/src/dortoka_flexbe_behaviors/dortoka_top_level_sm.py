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
class Dortoka_top_levelSM(Behavior):
    '''
    Top level behavior of the Dortoka Turtlebot
    '''


    def __init__(self):
        super(Dortoka_top_levelSM, self).__init__()
        self.name = 'Dortoka_top_level'

        # parameters of this behavior

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:587 y:44, x:584 y:159
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

        # x:30 y:365, x:130 y:365
        _sm_roamingcontainer_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_roamingcontainer_0:
            # x:147 y:101
            OperatableStateMachine.add('GoFwd',
                                        GoForwardState(speed=0.3, travel_dist=0.1, obstacle_dist=0.3),
                                        transitions={'failed': 'failed', 'done': 'Wait'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:466 y:100
            OperatableStateMachine.add('Wait',
                                        WaitState(wait_time=60),
                                        transitions={'done': 'GoFwd'},
                                        autonomy={'done': Autonomy.Off})


        # x:325 y:379, x:314 y:199
        _sm_gotodockprioritycontainer_1 = PriorityContainer(outcomes=['finished', 'failed'])

        with _sm_gotodockprioritycontainer_1:
            # x:59 y:32
            OperatableStateMachine.add('FindDock',
                                        FindDockState(),
                                        transitions={'failed': 'failed', 'charger_found': 'Autodock'},
                                        autonomy={'failed': Autonomy.Off, 'charger_found': Autonomy.Off})

            # x:63 y:339
            OperatableStateMachine.add('Charging',
                                        Charging(),
                                        transitions={'failed': 'failed', 'fully_charged': 'Undock', 'charging': 'WaitTillFullyCharged'},
                                        autonomy={'failed': Autonomy.Off, 'fully_charged': Autonomy.Off, 'charging': Autonomy.Off})

            # x:61 y:465
            OperatableStateMachine.add('Undock',
                                        GoForwardState(speed=-0.3, travel_dist=0.5, obstacle_dist=0.3),
                                        transitions={'failed': 'failed', 'done': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:65 y:233
            OperatableStateMachine.add('WaitTillFullyCharged',
                                        WaitState(wait_time=60),
                                        transitions={'done': 'Charging'},
                                        autonomy={'done': Autonomy.Off})

            # x:62 y:136
            OperatableStateMachine.add('Autodock',
                                        AutodockState(),
                                        transitions={'done': 'WaitTillFullyCharged', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


        # x:703 y:96, x:264 y:280
        _sm_batterymonitorcontainer_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_batterymonitorcontainer_2:
            # x:113 y:91
            OperatableStateMachine.add('BatteryMonitor',
                                        BatteryMonitor(battery_threshold=162),
                                        transitions={'failed': 'failed', 'low_battery': 'GoToDockPriorityContainer'},
                                        autonomy={'failed': Autonomy.Off, 'low_battery': Autonomy.Off})

            # x:351 y:86
            OperatableStateMachine.add('GoToDockPriorityContainer',
                                        _sm_gotodockprioritycontainer_1,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        # x:285 y:250, x:115 y:251, x:403 y:298, x:492 y:289, x:430 y:365
        _sm_toplevelcontainer_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
                                        ('finished', [('RoamingContainer', 'finished'), ('BatteryMonitorContainer', 'finished')]),
                                        ('failed', [('RoamingContainer', 'failed')]),
                                        ('failed', [('BatteryMonitorContainer', 'failed')])
                                        ])

        with _sm_toplevelcontainer_3:
            # x:127 y:84
            OperatableStateMachine.add('RoamingContainer',
                                        _sm_roamingcontainer_0,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:342 y:82
            OperatableStateMachine.add('BatteryMonitorContainer',
                                        _sm_batterymonitorcontainer_2,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



        with _state_machine:
            # x:256 y:68
            OperatableStateMachine.add('TopLevelContainer',
                                        _sm_toplevelcontainer_3,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
