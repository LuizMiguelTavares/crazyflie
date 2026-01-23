#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class CFJoyTwist:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.axis_roll   = rospy.get_param('~axis_roll',   3) 
        self.axis_pitch  = rospy.get_param('~axis_pitch',  4)
        self.axis_yaw    = rospy.get_param('~axis_yaw',    0)
        self.axis_thrust = rospy.get_param('~axis_thrust', 1)

        self.k_roll   = rospy.get_param('~scale_roll',   1.0)
        self.k_pitch  = rospy.get_param('~scale_pitch',  1.0)
        self.k_yaw    = rospy.get_param('~scale_yaw',    1.0)
        self.k_thrust = rospy.get_param('~scale_thrust', 1.0)

        rospy.Subscriber('joy', Joy, self.cb)

    def cb(self, msg: Joy):

        t = Twist()
        t.angular.x = -self.k_roll   * msg.axes[self.axis_roll]
        t.angular.y = self.k_pitch  * msg.axes[self.axis_pitch]
        t.angular.z = self.k_yaw    * msg.axes[self.axis_yaw]
        t.linear.z  = self.k_thrust * msg.axes[self.axis_thrust]
        self.pub.publish(t)

if __name__ == '__main__':
    rospy.init_node('cf_joy_twist')
    CFJoyTwist()
    rospy.spin()
