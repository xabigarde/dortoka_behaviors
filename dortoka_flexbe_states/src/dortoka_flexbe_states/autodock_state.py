#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus


class AutodockState(EventState):
	'''
	Automatically docks
	'''

	def __init__(self):
		# See example_state.py for basic explanations.
		super(AutodockState, self).__init__(outcomes = ['done', 'failed'])

		# Create the action client when building the behavior.
		# This will cause the behavior to wait for the client before starting execution
		# and will trigger a timeout error if it is not available.
		# Using the proxy client provides asynchronous access to the result and status
		# and makes sure only one client is used, no matter how often this state is used in a behavior.
		self._topic = 'dock_drive_action'
		self._client = ProxyActionClient({self._topic: AutoDockingAction}) # pass required clients as dict (topic: type)

		# It may happen that the action client fails to send the action goal.
		self._error = False

	def doneCb(self, status, result):
		if 0:
			print ''
		elif status == GoalStatus.PENDING:
			state = 'PENDING'
		elif status == GoalStatus.ACTIVE:
			state = 'ACTIVE'
		elif status == GoalStatus.PREEMPTED:
			state = 'PREEMPTED'
		elif status == GoalStatus.SUCCEEDED:
			state = 'SUCCEEDED'
		elif status == GoalStatus.ABORTED:
			state = 'ABORTED'
		elif status == GoalStatus.REJECTED:
			state = 'REJECTED'
		elif status == GoalStatus.PREEMPTING:
			state = 'PREEMPTING'
		elif status == GoalStatus.RECALLING:
			state = 'RECALLING'
		elif status == GoalStatus.RECALLED:
			state = 'RECALLED'
		elif status == GoalStatus.LOST:
			state = 'LOST'
		# Print state of action server
		print 'Result - [ActionServer: ' + state + ']: ' + result.text

	def activeCb(self):
		if 0: print 'Action server went active.'

	def feedbackCb(self, feedback):
		# Print state of dock_drive module (or node.)
		print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

	def execute(self, userdata):
		# While this state is active, check if the action has been finished and evaluate the result.

		# Check if the client failed to send the goal.
		if self._error:
			return 'command_error'

		# Check if the action has been finished
		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)

			return 'done'

		# If the action has not yet finished, no outcome will be returned and the state stays active.
		

	def on_enter(self, userdata):
		# When entering this state, we send the action goal once to let the robot start its work.

		# Create the goal.
		goal = AutoDockingGoal()

		# Send the goal.
		self._error = False # make sure to reset the error state since a previous state execution might have failed
		try:
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
		
