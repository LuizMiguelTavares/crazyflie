#!/usr/bin/env python3
import time
import rospy

from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import String, Bool, Float32
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

class CrazyflieServerNode:
    def __init__(self):
        rospy.init_node('crazyflie_server_node')

        ################### CRAZYFLIE SETUP #######################
        cflib.crtp.init_drivers()
        self._cf = Crazyflie()

        # State flags
        self.supervisor_flag = False
        self.battery_flag    = False
        self.vel_flag        = False
        self.thrust_flag     = False
        self.ang_flag        = False
        self.acc_flag        = False
        self.gyro_raw_flag   = False
        self.gyro_flag       = False
        
        self.old_flow_time   = None

        # Crazyflie connection callbacks
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Choose between angle commands or body rate
        self.use_body_rate = rospy.get_param('~use_body_rate', False)
        
        # LOG params
        self.vel_LOG      = rospy.get_param('~vel_LOG', True)
        self.ang_LOG      = rospy.get_param('~ang_LOG', True)
        self.thrust_LOG   = rospy.get_param('~thrust_LOG', True)
        self.acc_LOG      = rospy.get_param('~acc_LOG', True)
        self.gyro_raw_LOG = rospy.get_param('~gyro_raw_LOG', True)
        self.gyro_LOG     = rospy.get_param('~gyro_LOG', True)
        self.is_running_with_copilot = rospy.get_param('~is_running_with_copilot', False)

        # Stabilizer controller/estimator from ROS params
        self.stabilizer_controller = rospy.get_param('~stabilizer_controller', 1)
        self.stabilizer_estimator  = rospy.get_param('~stabilizer_estimator', 3)

        # ID -> URI mapping
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
            raise Exception("Required parameter 'ID' not set")

        # Prepare the link
        uri = uri_helper.uri_from_env(default=uri_)
        self._cf.open_link(uri)

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        ################### CRAZYFLIE TOPICS #######################
        # Subscriber for setpoints
        if self.is_running_with_copilot:
            self.cmd_vel_subscriber = rospy.Subscriber("copilot_cmd_vel", Twist, self.publish_twist)
        else:
            self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.publish_twist)

        # Publishers for data
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
        
        # self.pub_supervisor       = rospy.Publisher('crazyflieSupervisor', String, queue_size=10)
        self.pub_is_flying        = rospy.Publisher('crazyflieIsFlying', Bool, queue_size=10)
        self.pub_can_fly          = rospy.Publisher('crazyflieCanFly', Bool, queue_size=10)
        self.pub_z_range          = rospy.Publisher('crazyflieZRange', Vector3Stamped, queue_size=10)
        self.pub_battery_voltage  = rospy.Publisher('crazyflieBatteryVoltage', Float32, queue_size=10)
        self.pub_battery_level    = rospy.Publisher('crazyflieBatteryLevel', Float32, queue_size=10)

    def _connected(self, link_uri):
        rospy.loginfo(f"Crazyflie {link_uri} connected!")
        rospy.Timer(rospy.Duration(2.0), self._initialize_params_and_logs, oneshot=True)

    def _initialize_params_and_logs(self, event):
        # 1) Set parameters
        try:
            self._cf.param.set_value("stabilizer.controller", self.stabilizer_controller)
            rospy.loginfo(f"Set stabilizer.controller to {self.stabilizer_controller}")
        except Exception as e:
            rospy.logerr(f"Error setting stabilizer.controller: {e}")

        try:
            self._cf.param.set_value("stabilizer.estimator", self.stabilizer_estimator)
            rospy.loginfo(f"Set stabilizer.estimator to {self.stabilizer_estimator}")
        except Exception as e:
            rospy.logerr(f"Error setting stabilizer.estimator: {e}")

        if self.use_body_rate:
            rospy.loginfo("Using Body Rate mode")
            try:
                self._cf.param.set_value("flightmode.stabModeRoll", 0)
                self._cf.param.set_value("flightmode.stabModePitch", 0)
                self._cf.param.set_value("flightmode.stabModeYaw", 0)
            except Exception as e:
                rospy.logerr(f"Error setting flightmode.stabMode*: {e}")
        else:
            rospy.loginfo("Using Angle mode")
            try:
                self._cf.param.set_value("flightmode.stabModeRoll", 1)
                self._cf.param.set_value("flightmode.stabModePitch", 1)
                self._cf.param.set_value("flightmode.stabModeYaw", 0) # Yaw should continue to be body rate
            except Exception as e:
                rospy.logerr(f"Error setting flightmode.stabMode*: {e}")

        # 2) Setup logs
        self._setup_logs()

    def _setup_logs(self):
        self._lg_supervisor = LogConfig(name='Supervisor', period_in_ms=1000/1)
        self._lg_supervisor.add_variable('supervisor.info', 'uint16_t')

        self._lg_flow = LogConfig(name='Optical Flow', period_in_ms=1000/60)
        self._lg_flow.add_variable('motion.deltaX', 'float')
        self._lg_flow.add_variable('motion.deltaY', 'float')
        self._lg_flow.add_variable('range.zrange', 'float')

        self._lg_battery = LogConfig(name='Battery', period_in_ms=1000/1)
        self._lg_battery.add_variable('pm.vbat', 'float')
        self._lg_battery.add_variable('pm.batteryLevel', 'int8_t')

        if self.vel_LOG:
            self._lg_vel = LogConfig(name='Velocity', period_in_ms=1000/60)
            self._lg_vel.add_variable('stateEstimate.vx', 'float')
            self._lg_vel.add_variable('stateEstimate.vy', 'float')
            self._lg_vel.add_variable('stateEstimate.vz', 'float')
            self._lg_vel.add_variable('stateEstimate.z',  'float')

        if self.thrust_LOG:
            self._lg_thrust = LogConfig(name='Thrust', period_in_ms=1000/60)
            self._lg_thrust.add_variable('stabilizer.thrust', 'float')

        if self.ang_LOG:
            self._lg_ang = LogConfig(name='Angle', period_in_ms=1000/60)
            self._lg_ang.add_variable('stateEstimate.pitch', 'float')
            self._lg_ang.add_variable('stateEstimate.roll',  'float')
            self._lg_ang.add_variable('stateEstimate.yaw',   'float')

        if self.acc_LOG:
            self._lg_acc = LogConfig(name='Acceleration', period_in_ms=1000/60)
            self._lg_acc.add_variable('acc.x', 'float')
            self._lg_acc.add_variable('acc.y', 'float')
            self._lg_acc.add_variable('acc.z', 'float')

        if self.gyro_raw_LOG:
            self._lg_gyro_raw = LogConfig(name='GyroRaw', period_in_ms=1000/60)
            self._lg_gyro_raw.add_variable('gyro.xRaw', 'int16_t')
            self._lg_gyro_raw.add_variable('gyro.yRaw', 'int16_t')
            self._lg_gyro_raw.add_variable('gyro.zRaw', 'int16_t')

        if self.gyro_LOG:
            self._lg_gyro = LogConfig(name='Gyro', period_in_ms=1000/60)
            self._lg_gyro.add_variable('gyro.x', 'float')
            self._lg_gyro.add_variable('gyro.y', 'float')
            self._lg_gyro.add_variable('gyro.z', 'float')

        # Add & start
        self._start_log(self._lg_supervisor, 
                        self._supervisor_log_data, self._supervisor_log_error)
        self._start_log(self._lg_flow, 
                        self._flow_log_data, self._flow_log_error)
        self._start_log(self._lg_battery, 
                        self._battery_log_data, self._battery_log_error)

        if self.vel_LOG:
            self._start_log(self._lg_vel, self._vel_log_data, self._vel_log_error)
        if self.thrust_LOG:
            self._start_log(self._lg_thrust, self._thrust_data, self._thrust_log_error)
        if self.ang_LOG:
            self._start_log(self._lg_ang, self._ang_log_data, self._ang_log_error)
        if self.acc_LOG:
            self._start_log(self._lg_acc, self._acc_log_data, self._acc_log_error)
        if self.gyro_raw_LOG:
            self._start_log(self._lg_gyro_raw, self._gyro_raw_log_data, self._gyro_raw_log_error)
        if self.gyro_LOG:
            self._start_log(self._lg_gyro, self._gyro_log_data, self._gyro_log_error)

    def _start_log(self, logconf, data_cb, error_cb):
        try:
            self._cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(data_cb)
            logconf.error_cb.add_callback(error_cb)
            logconf.start()
        except KeyError as e:
            rospy.logerr(f"Could not start log config {logconf.name}, {str(e)} not found in TOC.")
        except AttributeError:
            rospy.logerr(f"Could not add log config {logconf.name}, bad configuration.")

    ############# Connection/Disconnection Callbacks #############
    def _connection_failed(self, link_uri, msg):
        rospy.loginfo(f"Connection to {link_uri} failed: {msg}")

    def _connection_lost(self, link_uri, msg):
        rospy.loginfo(f"Connection to {link_uri} lost: {msg}")

    def _disconnected(self, link_uri):
        rospy.loginfo(f"Disconnected from {link_uri}")

    ############# LOG ERROR CALLS #############
    def _supervisor_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _flow_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _battery_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _vel_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _thrust_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _ang_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _acc_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _gyro_raw_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")
    def _gyro_log_error(self, logconf, msg):
        rospy.logerr(f"Error in {logconf.name} log: {msg}")

    ############# LOG DATA CALLBACKS #############
    def _supervisor_log_data(self, timestamp, data, logconf):
        if not self.supervisor_flag:
            rospy.loginfo("Supervisor log started")
            self.supervisor_flag = True
        supervisor_info = data.get('supervisor.info', 0)

        # Bits
        # bit_0 = (supervisor_info >> 0) & 1  # can be armed
        # bit_1 = (supervisor_info >> 1) & 1  # is armed
        # bit_2 = (supervisor_info >> 2) & 1  # auto arm
        bit_3 = (supervisor_info >> 3) & 1  # can fly
        bit_4 = (supervisor_info >> 4) & 1  # is flying
        # bit_5 = (supervisor_info >> 5) & 1  # is tumbled
        # bit_6 = (supervisor_info >> 6) & 1  # is locked

        # string_0 = f"Can be armed={bool(bit_0)}, "
        # string_1 = f"Is armed={bool(bit_1)}, "
        # string_2 = f"Auto arm={bool(bit_2)}, "
        # string_3 = f"Can fly={bool(bit_3)}, "
        # string_4 = f"Is flying={bool(bit_4)}, "
        # string_5 = f"Is tumbled={bool(bit_5)}, "
        # string_6 = f"Is locked={bool(bit_6)}"

        # string_message = string_0 + string_1 + string_2 + string_3 + string_4 + string_5 + string_6
        # self.pub_supervisor.publish(string_message)

        self.pub_can_fly.publish(bool(bit_3))
        self.pub_is_flying.publish(bool(bit_4))

    def _flow_log_data(self, timestamp, data, logconf):
        # delta_x = data.get('motion.deltaX', 0)
        # delta_y = data.get('motion.deltaY', 0)
        z_range = data.get('range.zrange', 0) / 1000.0  # mm to m

        # if self.old_flow_time is None:
        #     self.old_flow_time = timestamp
        #     return

        # delta_t_ms = float(timestamp - self.old_flow_time)
        # self.old_flow_time = timestamp
        # dt_s = delta_t_ms / 1000.0

        # # Example direct scale
        # v_x = 0.0
        # v_y = 0.0
        # if dt_s > 0:
        #     v_x = -delta_y * z_range / dt_s
        #     v_y = -delta_x * z_range / dt_s

        # Publish z_range
        z_range_msg = Vector3Stamped()
        z_range_msg.header.stamp = rospy.Time.now()
        z_range_msg.vector.z = z_range
        self.pub_z_range.publish(z_range_msg)

        # Optionally, you could publish flow velocity too if desired.
        # flow_vel = Vector3Stamped()
        # flow_vel.header.stamp = rospy.Time.now()
        # flow_vel.vector.x = v_x
        # flow_vel.vector.y = v_y
        # self.pub_flow_vel.publish(flow_vel)

    def _battery_log_data(self, timestamp, data, logconf):
        if not self.battery_flag:
            rospy.loginfo("Battery log started")
            self.battery_flag = True
        battery_voltage = data.get('pm.vbat', 0)
        battery_level   = data.get('pm.batteryLevel', 0)

        # Publish
        battery_voltage_msg = Float32(data=battery_voltage)
        self.pub_battery_voltage.publish(battery_voltage_msg)

        battery_level_msg = Float32(data=float(battery_level))
        self.pub_battery_level.publish(battery_level_msg)

    def _vel_log_data(self, timestamp, data, logconf):
        if not self.vel_flag:
            rospy.loginfo("Velocity log started")
            self.vel_flag = True

        vx = data.get('stateEstimate.vx', 0)
        vy = data.get('stateEstimate.vy', 0)
        vz = data.get('stateEstimate.vz', 0)

        # Publish
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.vector.x = vx
        vel_msg.vector.y = vy
        vel_msg.vector.z = vz
        self.pub_vel.publish(vel_msg)

    def _thrust_data(self, timestamp, data, logconf):
        if not self.thrust_flag:
            rospy.loginfo("Thrust log started")
            self.thrust_flag = True
        thrust = data.get('stabilizer.thrust', 0)

        # Publish
        thrust_msg = Vector3Stamped()
        thrust_msg.header.stamp = rospy.Time.now()
        thrust_msg.vector.z = thrust
        self.pub_thrust.publish(thrust_msg)

    def _ang_log_data(self, timestamp, data, logconf):
        if not self.ang_flag:
            rospy.loginfo("Angle log started")
            self.ang_flag = True
        roll_deg  = data.get('stateEstimate.roll', 0)
        pitch_deg = data.get('stateEstimate.pitch', 0)
        yaw_deg   = data.get('stateEstimate.yaw', 0)

        # Convert deg -> rad
        roll_rad  = roll_deg  * 0.0174533
        pitch_rad = pitch_deg * 0.0174533
        yaw_rad   = yaw_deg   * 0.0174533

        # Publish
        ang_msg = Vector3Stamped()
        ang_msg.header.stamp = rospy.Time.now()
        ang_msg.vector.x = roll_rad
        ang_msg.vector.y = -pitch_rad 
        ang_msg.vector.z = yaw_rad
        self.pub_ang.publish(ang_msg)

    def _acc_log_data(self, timestamp, data, logconf):
        if not self.acc_flag:
            rospy.loginfo("Acceleration log started")
            self.acc_flag = True
        acc_x = data.get('acc.x', 0)
        acc_y = data.get('acc.y', 0)
        acc_z = data.get('acc.z', 0)

        # Convert from sensor units to m/s^2
        gravity = 9.81
        acc_x_m_s2 = -acc_x * gravity
        acc_y_m_s2 = -acc_y * gravity
        acc_z_m_s2 = -acc_z * gravity

        # Publish
        acc_msg = Vector3Stamped()
        acc_msg.header.stamp = rospy.Time.now()
        acc_msg.vector.x = acc_x_m_s2
        acc_msg.vector.y = acc_y_m_s2
        acc_msg.vector.z = acc_z_m_s2
        self.pub_acc.publish(acc_msg)

    def _gyro_raw_log_data(self, timestamp, data, logconf):
        if not self.gyro_raw_flag:
            rospy.loginfo("Raw Gyro log started")
            self.gyro_raw_flag = True
        gx_raw = data.get('gyro.xRaw', 0)
        gy_raw = data.get('gyro.yRaw', 0)
        gz_raw = data.get('gyro.zRaw', 0)

        # Convert from int16 to rad/s
        gx_rad_s = gx_raw * 0.001065
        gy_rad_s = gy_raw * 0.001065
        gz_rad_s = gz_raw * 0.001065

        # Publish
        gyro_raw_msg = Vector3Stamped()
        gyro_raw_msg.header.stamp = rospy.Time.now()
        gyro_raw_msg.vector.x = gx_rad_s
        gyro_raw_msg.vector.y = gy_rad_s
        gyro_raw_msg.vector.z = gz_rad_s
        self.pub_raw_gyro.publish(gyro_raw_msg)

    def _gyro_log_data(self, timestamp, data, logconf):
        if not self.gyro_flag:
            rospy.loginfo("Gyro log started")
            self.gyro_flag = True
        gx_deg_s = data.get('gyro.x', 0)
        gy_deg_s = data.get('gyro.y', 0)
        gz_deg_s = data.get('gyro.z', 0)

        # deg/s -> rad/s
        gx_rad_s = gx_deg_s * 0.0174533
        gy_rad_s = gy_deg_s * 0.0174533
        gz_rad_s = gz_deg_s * 0.0174533

        # Publish
        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp = rospy.Time.now()
        gyro_msg.vector.x = gx_rad_s
        gyro_msg.vector.y = gy_rad_s
        gyro_msg.vector.z = gz_rad_s
        self.pub_gyro.publish(gyro_msg)

    ############# CMD_VEL Callback #############
    def publish_twist(self, msg):
        """Receive a Twist and send setpoints to the Crazyflie."""
        if self.use_body_rate:
            rollrate  = msg.angular.x  * 57.2958
            pitchrate = msg.angular.y  * 57.2958
            yawrate   = msg.angular.z  * 57.2958
            thrust    = int(msg.linear.z)

            self._cf.commander.send_setpoint(
                rollrate, pitchrate, -yawrate, thrust
            )
        else:
            roll    = msg.angular.x * 57.2958
            pitch   = msg.angular.y * 57.2958
            yawrate = msg.angular.z * 57.2958
            thrust  = int(msg.linear.z)

            self._cf.commander.send_setpoint(
                roll, pitch, -yawrate, thrust
            )

    def _connection_failed(self, link_uri, msg):
        rospy.loginfo(f"Connection to {link_uri} failed: {msg}")

    def _connection_lost(self, link_uri, msg):
        rospy.loginfo(f"Connection to {link_uri} lost: {msg}")

    def _disconnected(self, link_uri):
        rospy.loginfo(f"Disconnected from {link_uri}")

if __name__ == '__main__':
    node = CrazyflieServerNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)
        rospy.signal_shutdown("Unhandled exception in CrazyflieServerNode")
    finally:
        node._cf.close_link()