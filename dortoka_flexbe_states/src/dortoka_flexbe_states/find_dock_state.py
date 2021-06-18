#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from kobuki_msgs.msg import DockInfraRed


class FindDockState(EventState):
    def __init__(self):
        super(FindDockState, self).__init__(outcomes=['failed', 'charger_found'])
        self.data = None

        self._last_ir_data = None

        self.dock_ir_topic = '/mobile_base/sensors/dock_ir'

        self.sensor_subscriber = ProxySubscriberCached({self.dock_ir_topic: DockInfraRed})
        self.sensor_subscriber.set_callback(self.dock_ir_topic, self.sensor_callback)

        Logger.loginfo("FindDock state initialized")

    def execute(self, userdata):
        if self.data is not None:

            if self.data.data != self._last_ir_data:
                Logger.loginfo("Dock IR receivers [Right, Center, Left]: %s" % self.data.data)
                self._last_ir_data = self.data
            if self.data.data != [0, 0, 0]:
                return 'charger_found'
            else:
                # TODO
                Logger.loginfo("TODO: roam towards dock")

    def on_enter(self, userdata):
        Logger.loginfo('FindDock state STARTED:')

    def on_exit(self, userdata):
        #self.sensor_subscriber.unsubscribe_topic(self.dock_ir_topic)
        Logger.loginfo('FindDock state ENDED')

    def on_start(self): 
        Logger.loginfo('FindDock state READY!')

    def on_stop(self): 
        Logger.loginfo('FindDock state STOPPED!')

    def sensor_callback(self, data):
        self.data = data
        self.sensor_subscriber.remove_last_msg(self.dock_ir_topic, True)
