#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from kobuki_msgs.msg import SensorState


class Charging(EventState):
    def __init__(self):
        super(Charging, self).__init__(outcomes=['failed', 'fully_charged', 'charging'])
        self.data = None

        self.core_sensors_topic = '/mobile_base/sensors/core'

        self.sensor_subscriber = ProxySubscriberCached({self.core_sensors_topic: SensorState})
        self.sensor_subscriber.set_callback(self.core_sensors_topic, self.sensor_callback)

    def execute(self, userdata):
        if self.data is not None:

            if self.data.charger == SensorState.DOCKING_CHARGED:
                charger_status = "DOCKING_CHARGED"
            elif self.data.charger == SensorState.DOCKING_CHARGING:
                charger_status = "DOCKING_CHARGING"
            elif self.data.charger == SensorState.ADAPTER_CHARGED:
                charger_status = "ADAPTER_CHARGED"
            elif self.data.charger == SensorState.ADAPTER_CHARGING:
                charger_status = "ADAPTER_CHARGING"

            Logger.loginfo("Charging status is: %s" % charger_status)
            if charger_status == SensorState.DOCKING_CHARGED or charger_status == SensorState.ADAPTER_CHARGED:
                return 'fully_charged'
            else:
                return 'charging'

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
