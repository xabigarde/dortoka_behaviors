#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class GoForwardState(EventState): 
    def __init__(self, speed, travel_dist, obstacle_dist): 
        super(GoForwardState, self).__init__(outcomes=['failed','done'])
        self._start_time = None 
        self.data = None
        self._speed = speed
        self._travel_dist = travel_dist
        self._obstacle_dist = obstacle_dist

        self.vel_topic = '/cmd_vel_mux/input/teleop'
        self.scan_topic = '/scan'

        self.pub = ProxyPublisher({self.vel_topic:Twist})
        self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
        self.scan_sub.set_callback(self.scan_topic, self.scan_callback)

    def execute(self, userdata): 
        if not self.cmd_pub:
            return 'failed'

        if (self.data is not None):
            Logger.loginfo("PWD obstacle distance is: %s" % self.data.ranges[424])
            if self.data.ranges[424] <= self._obstacle_dist: 
                return 'failed'

        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        distance_travelled = (elapsed_time) * self._speed

        Logger.loginfo("Distance travelled: %s" % distance_travelled)

        if distance_travelled >= self._travel_dist: 
                return 'done'

        self.pub.publish(self.vel_topic, self.cmd_pub)

    def on_enter(self, userdata):
        Logger.loginfo('Drive FWD STARTED:')

        self.cmd_pub = Twist()
        self.cmd_pub.linear.x = self._speed
        self.cmd_pub.angular.z = 0.0
        self._start_time = rospy.Time.now()
    

    def on_exit(self, userdata): 
        self.cmd_pub.linear.x = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        self.scan_sub.unsubscribe_topic(self.scan_topic)
        Logger.loginfo('Drive FWD ENDED')


    def on_start(self): 
        Logger.loginfo('Drive PWD READY!')

    def on_stop(self): 
        Logger.loginfo('Drive PWD STOPPED!')

    def scan_callback(self, data): 
        self.data = data
