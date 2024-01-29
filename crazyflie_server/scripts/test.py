#!/usr/bin/env python3
import rospy
from crazyflie_msgs.msg import CrazyflieLog

def talker():
    rospy.init_node('crazyflie_log_tester', anonymous=True)
    pub = rospy.Publisher('crazyflie_log_test', CrazyflieLog, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        log_msg = CrazyflieLog()
        log_msg.timestamp = rospy.Time.now()
        log_msg.vx = 1.0
        log_msg.vy = 2.0
        log_msg.vz = 3.0
        log_msg.z = 4.0
        log_msg.pitch = 5.0
        log_msg.roll = 6.0
        log_msg.thrust = 7.0

        rospy.loginfo(log_msg)
        pub.publish(log_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
