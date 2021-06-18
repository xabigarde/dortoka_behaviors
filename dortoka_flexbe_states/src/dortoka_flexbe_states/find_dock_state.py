#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from kobuki_msgs.msg import DockInfraRed


class FindDockState(EventState):
    def __init__(self):
        super(FindDockState, self).__init__(outcomes=['failed', 'charger_found'])
        self.data = None

        self.dock_ir_topic = '/mobile_base/sensors/dock_ir'

        self.sensor_subscriber = ProxySubscriberCached({self.core_sensors_topic: DockInfraRed})
        self.sensor_subscriber.set_callback(self.core_sensors_topic, self.sensor_callback)

    def execute(self, userdata):
        if self.data is not None:
            Logger.loginfo("Dock IR receivers [Right, Center, Left]: %s" % self.data)
            if self.data != [0, 0, 0]:
                return 'charger_found'

    def on_enter(self, userdata):
        Logger.loginfo('Drive FWD STARTED:')

    def on_exit(self, userdata):
        self.sensor_subscriber.unsubscribe_topic(self.core_sensors_topic)
        Logger.loginfo('BatteryMonitor ENDED')

    def on_start(self): 
        Logger.loginfo('BatteryMonitor READY!')

    def on_stop(self): 
        Logger.loginfo('BatteryMonitor STOPPED!')

    def sensor_callback(self, data):
        self.data = data
