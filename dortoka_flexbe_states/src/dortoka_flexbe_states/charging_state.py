#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from kobuki_msgs.msg import SensorState


class Charging(EventState):
    def __init__(self):
        super(Charging, self).__init__(outcomes=['failed', 'fully_charged', 'charging', 'discharging'])
        self.data = None
        self.kk = None
        self.last_charger_status = None

        self.core_sensors_topic = '/mobile_base/sensors/core'

        self.sensor_subscriber = ProxySubscriberCached({self.core_sensors_topic: SensorState})
        self.sensor_subscriber.set_callback(self.core_sensors_topic, self.sensor_callback)

        Logger.loginfo("Charging sate initialized")

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
            elif self.data.charger == SensorState.DISCHARGING:
                charger_status = "DISCHARGING"
            else:
                charger_status = "UNKNOWN"

            if charger_status != self.last_charger_status:
                Logger.loginfo("Charger topic value is: %s" % self.data.charger)
                Logger.loginfo("Charging status is: %s" % charger_status)
                self.last_charger_status = charger_status

            if self.data.charger == SensorState.DOCKING_CHARGED or self.data.charger == SensorState.ADAPTER_CHARGED:
                return 'fully_charged'
            elif self.data.charger == SensorState.DISCHARGING:
                return 'discharging'
            elif self.data.charger == SensorState.DOCKING_CHARGING or self.data.charger == SensorState.ADAPTER_CHARGING:
                self.kk = None
                #return 'charging'
            else:
                return 'failed'

    def on_enter(self, userdata):
        Logger.loginfo('Charging state STARTED:')
        #self.sensor_subscriber.subscribe(self.core_sensors_topic, SensorState, self.sensor_callback())

    def on_exit(self, userdata):
        #self.sensor_subscriber.unsubscribe_topic(self.core_sensors_topic)
        Logger.loginfo('Charging state ENDED')

    def on_start(self): 
        Logger.loginfo('Charging state READY!')

    def on_stop(self): 
        Logger.loginfo('Charging state STOPPED!')

    def sensor_callback(self, data):
        self.data = data
        self.sensor_subscriber.remove_last_msg(self.core_sensors_topic, True)
        #self.sensor_subscriber.remove_last_msg(self.core_sensors_topic)
