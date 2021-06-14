#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dortoka_flexbe_states.go_forward_state import GoForwardState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 11 2021
@author: Xabi
'''
class simple_go_forwardSM(Behavior):
	'''
	enalbes robot to go fwd
	'''


	def __init__(self):
		super(simple_go_forwardSM, self).__init__()
		self.name = 'simple_go_forward'

		# parameters of this behavior
		self.add_parameter('my_speed', 0.4)
		self.add_parameter('travel_distance', 0.3)
		self.add_parameter('obstacle_distance', 1.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:347 y:60
			OperatableStateMachine.add('Drive',
										GoForwardState(speed=self.my_speed, travel_dist=self.travel_distance, obstacle_dist=self.obstacle_distance),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
