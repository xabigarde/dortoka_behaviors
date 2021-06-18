#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dortoka_flexbe_behaviors.batterymonitor_sm import BatteryMonitorSM
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
        self.add_behavior(BatteryMonitorSM, 'TopLevelConcurrentContainer/BatteryMonitor')

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
                                        GoForwardState(speed=0.2, travel_dist=0.1, obstacle_dist=0.3),
                                        transitions={'failed': 'failed', 'done': 'Wait'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:466 y:100
            OperatableStateMachine.add('Wait',
                                        WaitState(wait_time=1),
                                        transitions={'done': 'GoBackwd'},
                                        autonomy={'done': Autonomy.Off})

            # x:236 y:266
            OperatableStateMachine.add('Wait2',
                                        WaitState(wait_time=1),
                                        transitions={'done': 'GoFwd'},
                                        autonomy={'done': Autonomy.Off})

            # x:432 y:326
            OperatableStateMachine.add('GoBackwd',
                                        GoForwardState(speed=-0.2, travel_dist=0.1, obstacle_dist=0.3),
                                        transitions={'failed': 'failed', 'done': 'Wait2'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


        # x:285 y:250, x:115 y:251, x:403 y:298, x:492 y:289, x:430 y:365, x:530 y:465
        _sm_toplevelconcurrentcontainer_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
                                        ('finished', [('RoamingContainer', 'finished')]),
                                        ('failed', [('RoamingContainer', 'failed')]),
                                        ('finished', [('BatteryMonitor', 'finished')]),
                                        ('failed', [('BatteryMonitor', 'failed')])
                                        ])

        with _sm_toplevelconcurrentcontainer_1:
            # x:127 y:84
            OperatableStateMachine.add('RoamingContainer',
                                        _sm_roamingcontainer_0,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:454 y:87
            OperatableStateMachine.add('BatteryMonitor',
                                        self.use_behavior(BatteryMonitorSM, 'TopLevelConcurrentContainer/BatteryMonitor'),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



        with _state_machine:
            # x:256 y:68
            OperatableStateMachine.add('TopLevelConcurrentContainer',
                                        _sm_toplevelconcurrentcontainer_1,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
