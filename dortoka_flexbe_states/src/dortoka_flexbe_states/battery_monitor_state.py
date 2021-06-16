#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from kobuki_msgs.msg import SensorState


class BatteryMonitor(EventState):
    def __init__(self, battery_threshold):
        super(BatteryMonitor, self).__init__(outcomes=['failed', 'low_battery'])
        self._battery_threshold = battery_threshold
        self.data = None

        self.core_sensors_topic = '/mobile_base/sensors/core'

        self.sensor_subscriber = ProxySubscriberCached({self.core_sensors_topic: SensorState})
        self.sensor_subscriber.set_callback(self.core_sensors_topic, self.sensor_callback)

    def execute(self, userdata):
        if self.data is not None:
            Logger.loginfo("Battery level is: %s" % self.data.battery)
            if self.data.battery <= self._battery_threshold:
                return 'low_battery'

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
