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
from dortoka_flexbe_states.turn_state import TurnState
from flexbe_states.calculation_state import CalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 14 2021
@author: xabi
'''
class dumb_patrolSM(Behavior):
	'''
	The robot will execute a patrol behavior, searching for the longest path it can find.
	'''


	def __init__(self):
		super(dumb_patrolSM, self).__init__()
		self.name = 'dumb_patrol'

		# parameters of this behavior
		self.add_parameter('turning_angle', 0)
		self.add_parameter('turning_speed', 0.4)
		self.add_parameter('drive_speed', 0.4)
		self.add_parameter('travel_distance', 4)
		self.add_parameter('obstacle_distance', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		self.scan_topic = '/camera/ir/image_raw/image_topics'
		self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
		self.scan_sub.set_callback(self.scan_topic, self.scan_callback)
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1213 y:47, x:827 y:351
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.in_direction = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:205 y:56
			OperatableStateMachine.add('find_free_path',
										CalculationState(calculation=self.find_free_path_angle),
										transitions={'done': 'Turn to free path'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'in_direction', 'output_value': 'new_direction'})

			# x:380 y:179
			OperatableStateMachine.add('Turn to free path',
										TurnState(turn_angle=self.turning_angle, t_speed=self.turning_speed),
										transitions={'done': 'Go Forward', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:765 y:57
			OperatableStateMachine.add('Go Forward',
										GoForwardState(speed=self.drive_speed, travel_dist=self.travel_distance, obstacle_dist=self.obstacle_distance),
										transitions={'failed': 'failed', 'done': 'find_free_path'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def find_free_path_angle(self):
		#TODO

	def scan_callback(self, data): 
		self.data = data
	# [/MANUAL_FUNC]
















