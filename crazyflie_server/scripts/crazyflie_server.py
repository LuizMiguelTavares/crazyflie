#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import  Twist, Vector3

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from crazyflie_msgs.msg import CrazyflieLog

class CrazyflieServerNode:
    def __init__(self):
        rospy.init_node('crazyflie_server_node')

        ################### CRAZYFLIE SETUP #######################
        # Initializing crazyflie
        cflib.crtp.init_drivers()
        self._cf = Crazyflie()

        self.vel_flag = False
        self.angles_flag = False
        self.thrust_flag = False
        self.ang_flag = False

        # Crazyflie connection callbacks
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)        

        ID = rospy.get_param('~ID', None)
        
        if ID == 1:
            uri_ = "radio://0/10/2M/E7E7E7E701"
        elif ID == 2:
            uri_ = "radio://0/10/2M/E7E7E7E702"
        elif ID == 3:
            uri_ = "radio://0/10/2M/E7E7E7E703"
        elif ID == 4:
            uri_ = "radio://0/10/2M/E7E7E7E704"
        elif ID == 5:
            uri_ = "radio://0/20/2M/E7E7E7E705"
        elif ID == 6:
            uri_ = "radio://0/20/2M/E7E7E7E706"
        elif ID == 7:
            uri_ = "radio://0/20/2M/E7E7E7E707"
        elif ID == 8:
            uri_ = "radio://0/50/2M/E7E7E7E708"
        else:
            raise Exception(f"Invalid ID: {ID}")    
            
        if not rospy.has_param('~ID'):
            raise Exception(f"Required parameter 'uri' not set")
        
        stabilizer_controller = rospy.get_param('~stabilizer_controller', 1)
        
        if stabilizer_controller == 1:
            rospy.loginfo("Stabilizer controller set to PID")
        elif stabilizer_controller == 2:
            rospy.loginfo("Stabilizer controller set to Mellinger")
        elif stabilizer_controller == 3:
            rospy.loginfo("Stabilizer controller set to INDI")
        elif stabilizer_controller == 4:
            rospy.loginfo("Stabilizer controller set to Brescianini")
        else:
            raise Exception(f"Invalid stabilizer controller: {stabilizer_controller}")
        
        stabilizer_esimator = rospy.get_param('~stabilizer_estimator', 3)
        
        if stabilizer_esimator == 1:
            rospy.loginfo("Stabilizer estimator set to Complementary filter")
        elif stabilizer_esimator == 2:
            rospy.loginfo("Stabilizer estimator set to EKF")
        elif stabilizer_esimator == 3:
            rospy.loginfo("Stabilizer estimator set to unscented Kalman filter")
        else:
            raise Exception(f"Invalid stabilizer estimator: {stabilizer_esimator}")
        
        # Opening crazyflie link
        uri = uri_helper.uri_from_env(default=uri_)
        self._cf.open_link(uri)

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        ################### CRAZYFLIE PARAMETERS #######################

        # Setting Crazyflie parameters
        self._cf.param.set_value("stabilizer.controller", stabilizer_controller)  # 1 = PID, 2 = Mellinger, 3 = INDI, 4 = Brescianini
        self._cf.param.set_value("stabilizer.estimator", stabilizer_esimator)   # 1 = Complementary filter, 2 = EKF, 3 = unscented Kalman filter
        self._cf.param.set_value("ctrlMel.ki_m_z", 0)
        self._cf.param.set_value("ctrlMel.i_range_m_z", 0)
        self._cf.param.set_value("pid_attitude.pitch_ki", 0)
        self._cf.param.set_value("pid_attitude.roll_ki", 0)
        self._cf.param.set_value("pid_attitude.yaw_ki", 0)

        ################### CRAZYFLIE TOPICS #######################
        self.pose_subscriber = rospy.Subscriber(f"cmd_vel",
                                                Twist, 
                                                self.publish_twist)
        
        # Publishers for different sets of data
        self.pub_log = rospy.Publisher('crazyflieLog', CrazyflieLog, queue_size=10)
        self.pub_only_vel = rospy.Publisher('crazyflieVel', Vector3, queue_size=10)
        self.pub_only_ang = rospy.Publisher('crazyflieAng', Twist, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0/60.0), self._send_log_data)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        rospy.loginfo(f'Crazyflie {link_uri} connected!')

        self._lg_vel = LogConfig(name='velocity in the drone frame', period_in_ms=1000/60)
        self._lg_vel.add_variable('stateEstimate.vx', 'float')
        self._lg_vel.add_variable('stateEstimate.vy', 'float')
        self._lg_vel.add_variable('stateEstimate.vz', 'float')
        self._lg_vel.add_variable('stateEstimate.z', 'float')
        
        self._lg_ang = LogConfig(name='Angle of the drone', period_in_ms=1000/60)
        self._lg_ang.add_variable('stateEstimate.pitch', 'float')
        self._lg_ang.add_variable('stateEstimate.roll', 'float')
        self._lg_ang.add_variable('stateEstimate.yaw', 'float')

        self._lg_thrust = LogConfig(name='thrust', period_in_ms=1000/60)
        self._lg_thrust.add_variable('stabilizer.thrust', 'float')
        
        try:
            self._cf.log.add_config(self._lg_vel)
            self._cf.log.add_config(self._lg_thrust)
            self._cf.log.add_config(self._lg_ang)
            # This callback will receive the data
            self._lg_vel.data_received_cb.add_callback(self._vel_log_data)
            self._lg_thrust.data_received_cb.add_callback(self._thrust_data)
            self._lg_ang.data_received_cb.add_callback(self._ang_log_data)
            # This callback will be called on errors
            self._lg_vel.error_cb.add_callback(self._vel_log_error)
            self._lg_thrust.error_cb.add_callback(self._vel_log_error)
            self._lg_ang.error_cb.add_callback(self._vel_log_error)
            # Start the logging
            self._lg_vel.start()
            self._lg_thrust.start()
            self._lg_ang.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        rospy.loginfo('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.loginfo('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        rospy.loginfo('Disconnected from %s' % link_uri)

    def _vel_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _thrust_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _ang_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _vel_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self.vel_flag:
            rospy.loginfo("Velocity log started")
            self.vel_flag = True

        self.vx = data.get('stateEstimate.vx', 0)
        self.vy = data.get('stateEstimate.vy', 0)
        self.vz = data.get('stateEstimate.vz', 0)
        self.z = data.get('stateEstimate.z', 0)  
        # self.pitch = data.get('stateEstimate.pitch', 0)
        # self.roll = data.get('stateEstimate.roll', 0)
        # self.yaw = data.get('stateEstimate.yaw', 0)

    def _thrust_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self.thrust_flag:
            rospy.loginfo("Thrust log started")
            self.thrust_flag = True
        self.thrust = data.get('stabilizer.thrust', 0)
    
    def _ang_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self.ang_flag:
            rospy.loginfo("Angle log started")
            self.ang_flag = True
        self.pitch = data.get('stateEstimate.pitch', 0)
        self.roll = data.get('stateEstimate.roll', 0)
        self.yaw = data.get('stateEstimate.yaw', 0)

    def _send_log_data(self, event):
        if self.vel_flag and self.thrust_flag and self.ang_flag:
            log_msg = CrazyflieLog()
            vel_msg = Vector3()
            ang_msg = Twist()

            timestamp = rospy.Time.now().to_sec()
            log_msg.timestamp = timestamp

            log_msg.vx = self.vx
            log_msg.vy = self.vy    
            log_msg.vz = self.vz
            log_msg.z = self.z
            log_msg.pitch = self.pitch
            log_msg.roll = self.roll
            log_msg.thrust = self.thrust
            log_msg.yaw = self.yaw
        
            vel_msg.x = self.vx
            vel_msg.y = self.vy
            vel_msg.z = self.vz

            ang_msg.linear.x = self.pitch
            ang_msg.linear.y = self.roll
            ang_msg.linear.z = self.thrust
            ang_msg.angular.z = self.yaw

            self.pub_log.publish(log_msg)
            self.pub_only_vel.publish(vel_msg)
            self.pub_only_ang.publish(ang_msg)

    def publish_twist(self, msg):        
        roll = msg.linear.y
        pitch = msg.linear.x
        thrust = msg.linear.z
        yawrate = msg.angular.z
        self._cf.commander.send_setpoint(-roll, pitch, -yawrate, int(thrust))

if __name__ == '__main__':
    node = CrazyflieServerNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)
        rospy.signal_shutdown("Parameter not set")

    node._cf.close_link()