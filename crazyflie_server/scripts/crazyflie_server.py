#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import  Twist, Vector3Stamped
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
        self.thrust_flag = False
        self.ang_flag = False
        self.acc_flag = False
        self.gyro_raw_flag = False
        self.gyro_flag = False

        # Crazyflie connection callbacks
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost) 
        
        # Choose between ang or body rate
        self.use_body_rate = rospy.get_param('~use_body_rate', False)
        
        # LOG params
        # self.crazyflie_LOG = rospy.get_param('~crazyflie_LOG', True)
        self.vel_LOG = rospy.get_param('~vel_LOG', True)
        self.ang_LOG = rospy.get_param('~ang_LOG', True)
        self.thrust_LOG = rospy.get_param('~thrust_LOG', True)
        self.acc_LOG = rospy.get_param('~acc_LOG', True)
        self.gyro_raw_LOG = rospy.get_param('~gyro_raw_LOG', True)
        self.gyro_LOG = rospy.get_param('~gyro_LOG', True)
              
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
        
        stabilizer_estimator = rospy.get_param('~stabilizer_estimator', 3)
        
        if stabilizer_estimator == 1:
            rospy.loginfo("Stabilizer estimator set to Complementary filter")
        elif stabilizer_estimator == 2:
            rospy.loginfo("Stabilizer estimator set to EKF")
        elif stabilizer_estimator == 3:
            rospy.loginfo("Stabilizer estimator set to unscented Kalman filter")
        else:
            raise Exception(f"Invalid stabilizer estimator: {stabilizer_estimator}")
        
        # Opening crazyflie link
        uri = uri_helper.uri_from_env(default=uri_)
        self._cf.open_link(uri)

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        ################### CRAZYFLIE PARAMETERS #######################

        # Setting Crazyflie parameters
        self._cf.param.set_value("stabilizer.controller", stabilizer_controller)  # 1 = PID, 2 = Mellinger, 3 = INDI, 4 = Brescianini
        self._cf.param.set_value("stabilizer.estimator", stabilizer_estimator)   # 1 = Complementary filter, 2 = EKF, 3 = unscented Kalman filter
        # self._cf.param.set_value("ctrlMel.ki_m_z", 0)
        # self._cf.param.set_value("ctrlMel.i_range_m_z", 0)
        # self._cf.param.set_value("pid_attitude.pitch_ki", 0)
        # self._cf.param.set_value("pid_attitude.roll_ki", 0)
        # self._cf.param.set_value("pid_attitude.yaw_ki", 0)
        
        if self.use_body_rate:
            self._cf.param.set_value("flightmode.stabModeRoll", 0)
            self._cf.param.set_value("flightmode.stabModePitch", 0)
            self._cf.param.set_value("flightmode.stabModeYaw", 0)
        else:
            self._cf.param.set_value("flightmode.stabModeRoll", 1)
            self._cf.param.set_value("flightmode.stabModePitch", 1)
            self._cf.param.set_value("flightmode.stabModeYaw", 1)

        ################### CRAZYFLIE TOPICS #######################
        self.pose_subscriber = rospy.Subscriber(f"cmd_vel",
                                                Twist,
                                                self.publish_twist)
        
        # Publishers for different sets of data
        if self.vel_LOG:
            self.pub_vel = rospy.Publisher('crazyflieVel', Vector3Stamped, queue_size=10)
        if self.ang_LOG:
            self.pub_ang = rospy.Publisher('crazyflieAng', Vector3Stamped, queue_size=10)
        if self.thrust_LOG:
            self.pub_thrust = rospy.Publisher('crazyflieThrust', Vector3Stamped, queue_size=10)
        if self.acc_LOG:
            self.pub_acc = rospy.Publisher('crazyflieAcc', Vector3Stamped, queue_size=10)
        if self.gyro_raw_LOG:
            self.pub_raw_gyro = rospy.Publisher('crazyflieRawAngRate', Vector3Stamped, queue_size=10)
        if self.gyro_LOG:
            self.pub_gyro = rospy.Publisher('crazyflieAngRate', Vector3Stamped, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0/60.0), self._send_log_data)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        rospy.loginfo(f'Crazyflie {link_uri} connected!')
        rospy.sleep(1)

        if self.thrust_LOG:
            self._lg_thrust = LogConfig(name='thrust', period_in_ms=1000/60)
            self._lg_thrust.add_variable('stabilizer.thrust', 'float')
            
            try:
                rospy.loginfo("Adding thrust log...")
                self._cf.log.add_config(self._lg_thrust)
                self._lg_thrust.data_received_cb.add_callback(self._thrust_data)
                self._lg_thrust.error_cb.add_callback(self._vel_log_error)
                self._lg_thrust.start()
            except KeyError as e:
                print('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')
                
        if self.ang_LOG:
            self._lg_ang = LogConfig(name='Angle of the drone', period_in_ms=1000/60)
            self._lg_ang.add_variable('stateEstimate.pitch', 'float')
            self._lg_ang.add_variable('stateEstimate.roll', 'float')
            self._lg_ang.add_variable('stateEstimate.yaw', 'float')

            try:
                rospy.loginfo("Adding angle log...")
                self._cf.log.add_config(self._lg_ang)
                self._lg_ang.data_received_cb.add_callback(self._ang_log_data)
                self._lg_ang.error_cb.add_callback(self._vel_log_error)
                self._lg_ang.start()
            except KeyError as e:
                print('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')
        
        if self.acc_LOG:
            self._lg_acc = LogConfig(name='Acceleration in the drone frame', period_in_ms=1000/60)
            self._lg_acc.add_variable('acc.x', 'float')
            self._lg_acc.add_variable('acc.y', 'float')
            self._lg_acc.add_variable('acc.z', 'float')

            try:
                rospy.loginfo("Adding acceleration log...")
                self._cf.log.add_config(self._lg_acc)
                self._lg_acc.data_received_cb.add_callback(self._acc_log_data)
                self._lg_acc.error_cb.add_callback(self._vel_log_error)
                self._lg_acc.start()
            except KeyError as e:
                print('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')
                
        if self.gyro_raw_LOG:
            self._lg_gyro_raw = LogConfig(name='Gyro Raw in the drone frame', period_in_ms=1000/60)
            self._lg_gyro_raw.add_variable('gyro.xRaw', 'int16_t')
            self._lg_gyro_raw.add_variable('gyro.yRaw', 'int16_t')
            self._lg_gyro_raw.add_variable('gyro.zRaw', 'int16_t')

            try:
                rospy.loginfo("Adding raw gyro log...")
                self._cf.log.add_config(self._lg_gyro_raw)
                self._lg_gyro_raw.data_received_cb.add_callback(self._gyro_raw_log_data)
                self._lg_gyro_raw.error_cb.add_callback(self._gyro_raw_log_error)
                self._lg_gyro_raw.start()
            except KeyError as e:
                print('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')
            
        
        if self.gyro_LOG:
            self._lg_gyro = LogConfig(name='Gyro in the drone frame', period_in_ms=1000/60)
            self._lg_gyro.add_variable('gyro.x', 'float')
            self._lg_gyro.add_variable('gyro.y', 'float')
            self._lg_gyro.add_variable('gyro.z', 'float')

            try:
                rospy.loginfo("Adding filtered gyro log...")
                self._cf.log.add_config(self._lg_gyro)  # Suspect
                self._lg_gyro.data_received_cb.add_callback(self._gyro_log_data)
                self._lg_gyro.error_cb.add_callback(self._gyro_log_error)
                self._lg_gyro.start()
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
    
    def _acc_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))
    
    def _gyro_raw_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))
        
    def _gyro_log_error(self, logconf, msg):
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
    
    def _gyro_raw_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self.gyro_raw_flag:
            rospy.loginfo("Raw Gyro log started")
            self.gyro_raw_flag = True
        self.gyro_x_raw = data.get('gyro.xRaw', 0)
        self.gyro_y_raw = data.get('gyro.yRaw', 0)
        self.gyro_z_raw = data.get('gyro.zRaw', 0)
        
    def _gyro_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self.gyro_flag:
            rospy.loginfo("Gyro log started")
            self.gyro_flag = True
        self.gyro_x = data.get('gyro.x', 0)
        self.gyro_y = data.get('gyro.y', 0)
        self.gyro_z = data.get('gyro.z', 0)
        
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
            
        self.roll = data.get('stateEstimate.roll', 0)
        self.pitch = data.get('stateEstimate.pitch', 0)
        self.yaw = data.get('stateEstimate.yaw', 0)
    
    def _acc_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self.acc_flag:
            rospy.loginfo("Acceleration log started")
            self.acc_flag = True
        self.acc_x = data.get('acc.x', 0)
        self.acc_y = data.get('acc.y', 0)
        self.acc_z = data.get('acc.z', 0)

    def _send_log_data(self, event):
        
        if self.ang_LOG and self.ang_flag:
            ang_msg = Vector3Stamped()
            ang_msg.header.stamp = rospy.Time.now()
            
            # deg to rad
            roll = self.roll * 0.0174533
            pitch = self.pitch * 0.0174533
            yaw = self.yaw * 0.0174533

            ang_msg.vector.x = roll
            ang_msg.vector.y = -pitch
            ang_msg.vector.z = yaw
            self.pub_ang.publish(ang_msg)
            
        if self.vel_LOG and self.vel_flag:
            vel_msg = Vector3Stamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.vector.x = self.vx
            vel_msg.vector.y = self.vy
            vel_msg.vector.z = self.vz
            self.pub_vel.publish(vel_msg)
            
        if self.thrust_LOG and self.thrust_flag:
            thrust_msg = Vector3Stamped()
            thrust_msg.header.stamp = rospy.Time.now()
            thrust_msg.vector.z = self.thrust
            self.pub_thrust.publish(thrust_msg)
        
        if self.acc_LOG and self.acc_flag:
            acc_msg = Vector3Stamped()
            acc_msg.header.stamp = rospy.Time.now()
            gravity = 9.81
            acc_msg.vector.x = -self.acc_x * gravity
            acc_msg.vector.y = -self.acc_y * gravity
            acc_msg.vector.z = -self.acc_z * gravity
            self.pub_acc.publish(acc_msg)
        
        if self.gyro_raw_LOG and self.gyro_raw_flag:
            gyro_raw_msg = Vector3Stamped()
            gyro_raw_msg.header.stamp = rospy.Time.now()
            
            # Int to rad/s
            gyro_x_raw = self.gyro_x_raw * 0.001065
            gyro_y_raw = self.gyro_y_raw * 0.001065
            gyro_z_raw = self.gyro_z_raw * 0.001065
        
            gyro_raw_msg.vector.x = gyro_x_raw
            gyro_raw_msg.vector.y = gyro_y_raw
            gyro_raw_msg.vector.z = gyro_z_raw
            self.pub_raw_gyro.publish(gyro_raw_msg)
        
        if self.gyro_LOG and self.gyro_flag:
            gyro_msg = Vector3Stamped()
            gyro_msg.header.stamp = rospy.Time.now()
            
            # deg/s to rad/s
            gyro_x = self.gyro_x * 0.0174533
            gyro_y = self.gyro_y * 0.0174533
            gyro_z = self.gyro_z * 0.0174533
            
            gyro_msg.vector.x = gyro_x
            gyro_msg.vector.y = gyro_y
            gyro_msg.vector.z = gyro_z

            self.pub_gyro.publish(gyro_msg)

    def publish_twist(self, msg):
        
        if self.use_body_rate:
            rollrate = msg.angular.x
            pitchrate = msg.angular.y
            yawrate = msg.angular.z
            thrust = msg.linear.z
            
            # Rad to deg
            rollrate = rollrate * 57.2958
            pitchrate = pitchrate * 57.2958
            yawrate = yawrate * 57.2958
            
            self._cf.commander.send_setpoint(rollrate, pitchrate, -yawrate, int(thrust))
        else:
            roll = msg.angular.x
            pitch = msg.angular.y
            yawrate = msg.angular.z
            thrust = msg.linear.z
            
            # Rad to deg
            roll = roll * 57.2958
            pitch = pitch * 57.2958
            yawrate = yawrate * 57.2958
            
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