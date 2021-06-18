#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal, AutoDockingResult
from actionlib_msgs.msg import GoalStatus


class AutodockState(EventState):
	'''
	Automatically docks
	'''

	def __init__(self, timeout):
		# See example_state.py for basic explanations.
		super(AutodockState, self).__init__(outcomes=['done', 'failed', 'timed_out'])

		# Create the action client when building the behavior.
		# This will cause the behavior to wait for the client before starting execution
		# and will trigger a timeout error if it is not available.
		# Using the proxy client provides asynchronous access to the result and status
		# and makes sure only one client is used, no matter how often this state is used in a behavior.
		self._topic = 'dock_drive_action'
		self._client = ProxyActionClient({self._topic: AutoDockingAction}) # pass required clients as dict (topic: type)
		self._timeout = timeout
		# It may happen that the action client fails to send the action goal.
		self._error = False

		self._start_time = None

		Logger.loginfo("AutoDock state initialized")

	# def doneCb(self, status, result):
	# 	if 0:
	# 		print ''
	# 	elif status == GoalStatus.PENDING:
	# 		state = 'PENDING'
	# 	elif status == GoalStatus.ACTIVE:
	# 		state = 'ACTIVE'
	# 	elif status == GoalStatus.PREEMPTED:
	# 		state = 'PREEMPTED'
	# 	elif status == GoalStatus.SUCCEEDED:
	# 		state = 'SUCCEEDED'
	# 	elif status == GoalStatus.ABORTED:
	# 		state = 'ABORTED'
	# 	elif status == GoalStatus.REJECTED:
	# 		state = 'REJECTED'
	# 	elif status == GoalStatus.PREEMPTING:
	# 		state = 'PREEMPTING'
	# 	elif status == GoalStatus.RECALLING:
	# 		state = 'RECALLING'
	# 	elif status == GoalStatus.RECALLED:
	# 		state = 'RECALLED'
	# 	elif status == GoalStatus.LOST:
	# 		state = 'LOST'
	# 	# Print state of action server
	# 	print 'Result - [ActionServer: ' + state + ']: ' + result.text
	#
	# def activeCb(self):
	# 	if 0: print 'Action server went active.'
	#
	# def feedbackCb(self, feedback):
	# 	# Print state of dock_drive module (or node.)
	# 	print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

	def execute(self, userdata):
		# While this state is active, check if the action has been finished and evaluate the result.

		# Check if the client failed to send the goal.
		if self._error:
			return 'failed'

		# Check if the action has been finished
		if self._client.has_feedback(self._topic):
			feedback = self._client.get_feedback(self._topic)

			##################################################
			####### UN/COMMENT FOR DEBUGGING:   ##############
			#
			# if 0:
			# 	print ''
			# elif feedback.state == 'IDLE':
			# 	state = 'IDLE'
			# elif feedback.state == 'DONE':
			# 	state = 'DONE'
			# elif feedback.state == 'DOCKED_IN':
			# 	state = 'DOCKED_IN'
			# elif feedback.state == 'BUMPED_DOCK':
			# 	state = 'BUMPED_DOCK'
			# elif feedback.state == 'SCAN':
			# 	state = 'SCAN'
			# elif feedback.state == 'FIND_STREAM':
			# 	state = 'FIND_STREAM'
			# elif feedback.state == 'GET_STREAM':
			# 	state = 'GET_STREAM'
			# elif feedback.state == 'ALIGNED':
			# 	state = 'ALIGNED'
			# elif feedback.state == 'ALIGNED_FAR':
			# 	state = 'ALIGNED_FAR'
			# elif feedback.state == 'ALIGNED_NEAR':
			# 	state = 'ALIGNED_NEAR'
			# elif feedback.state == 'UNKNOWN':
			# 	state = 'UNKNOWN'
			# elif feedback.state == 'LOST':
			# 	state = 'LOST'
			# else:
			# 	state = 'this_sucks'
			# Logger.loginfo("AutoDock feedback: %s" % feedback)
			##################################################

			if feedback.state in ('DOCKED_IN', 'DONE'):
				return 'done'
			elif feedback in ('LOST', 'UNKNOWN'):
				return 'failed'
			elif feedback.state in ('BUMPED_DOCK', 'BUMPED', 'SCAN', 'FIND_STREAM', 'GET_STREAM', 'ALIGNED', 'ALIGNED_FAR', 'ALIGNED_NEAR'):
				# TODO
				pass
			elif feedback == 'IDLE':
				# TODO
				pass

			elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
			if elapsed_time >= self._timeout:
				return 'timed_out'

	# If the action has not yet finished, no outcome will be returned and the state stays active.

	def on_enter(self, userdata):
		# When entering this state, we send the action goal once to let the robot start its work.

		# Create the goal.
		goal = AutoDockingGoal()

		# Send the goal.
		self._error = False # make sure to reset the error state since a previous state execution might have failed
		try:
			self._start_time = rospy.Time.now()
			self._client.send_goal(self._topic, goal)
		except Exception as e:
			# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
			Logger.logwarn('Failed to send the AutoDocking command:\n%s' % str(e))
			self._error = True

	def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')