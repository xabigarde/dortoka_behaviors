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
from flexbe_states.check_condition_state import CheckConditionState
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
		self.add_parameter('turning_angle', 90)
		self.add_parameter('turning_speed', 0.4)
		self.add_parameter('drive_speed', 0.4)
		self.add_parameter('travel_distance', 1.0)
		self.add_parameter('obstacle_distance', 0.4)
		self.add_parameter('total_turns', 4)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		#self.scan_topic = '/camera/ir/image_raw/image_topics'
		#self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
		#self.scan_sub.set_callback(self.scan_topic, self.scan_callback)
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:297 y:503, x:775 y:45
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.in_direction = 0
		_state_machine.userdata.turns_done = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:252 y:98
			OperatableStateMachine.add('Drive Forward',
										GoForwardState(speed=self.drive_speed, travel_dist=self.travel_distance, obstacle_dist=self.obstacle_distance),
										transitions={'failed': 'failed', 'done': 'Turn'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

			# x:840 y:217
			OperatableStateMachine.add('Turn',
										TurnState(turn_angle=self.turning_angle, t_speed=self.turning_speed),
										transitions={'done': 'keep on', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:258 y:307
			OperatableStateMachine.add('keep on',
										CheckConditionState(predicate=keep_turnin),
										transitions={'true': 'Drive Forward', 'false': 'finished'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'turns_done'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	#def find_free_path_angle(self):
		#TODO
	def keep_turnin(turns_done)
		_state_machine.userdata.turns_done = turns_done
		if _state_machine.userdata.turns_done < self.total_turns:
			return true
		return false

	def scan_callback(self, data): 
		self.data = data
	# [/MANUAL_FUNC]
