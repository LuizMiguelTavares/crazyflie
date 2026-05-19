#!/usr/bin/env python3

import time
import sys
import signal
import math

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, PoseStamped
from std_msgs.msg import Bool, Float32

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

def id_to_uri(cf_id: int) -> str:
    """Return a radio URI given a numeric Crazyflie ID (1…8)."""
    table = {
        1: "radio://0/10/2M/E7E7E7E701",
        2: "radio://0/10/2M/E7E7E7E702",
        3: "radio://0/10/2M/E7E7E7E703",
        4: "radio://0/10/2M/E7E7E7E704",
        5: "radio://0/20/2M/E7E7E7E705",
        6: "radio://0/20/2M/E7E7E7E706",
        7: "radio://0/20/2M/E7E7E7E707",
        8: "radio://0/50/2M/E7E7E7E708",
        9: "radio://0/80/2M/E7E7E7E709",
    }
    if cf_id not in table:
        raise ValueError(f"Invalid Crazyflie ID {cf_id}")
    return table[cf_id]

class CrazyflieServer:
    def __init__(self, cf_id: int):
        self.ns = f"/cf{cf_id}"
        self.cf_id = cf_id

        # ---------- parameters (read from /cf<ID>/ namespace) ----------
        # get = lambda name, default: rospy.get_param(f"{self.ns}/{name}", default)

        self.use_body_rate          = rospy.get_param("~use_body_rate", True)
        self.filt_cutoff            = rospy.get_param("~filt_cutoff", 10)
        self.outerLoopActive        = rospy.get_param("~outerLoopActive", True)
        self.pos_LOG                = rospy.get_param("~pos_LOG", True)
        self.vel_LOG                = rospy.get_param("~vel_LOG", False)
        self.ang_LOG                = rospy.get_param("~ang_LOG", False)
        self.thrust_LOG             = rospy.get_param("~thrust_LOG", False)
        self.acc_LOG                = rospy.get_param("~acc_LOG", False)
        self.gyro_raw_LOG           = rospy.get_param("~gyro_raw_LOG", False)
        self.gyro_LOG               = rospy.get_param("~gyro_LOG", False)
        self.stab_rate_LOG          = rospy.get_param("~stab_rate_LOG", True)
        self.control_time_us_LOG    = rospy.get_param("~control_time_us_LOG", True)
        self.is_running_with_copilot= rospy.get_param("~is_running_with_copilot", False)
        self.stabilizer_controller  = rospy.get_param("~stabilizer_controller", 1) # 1 = PID, 2 = Mellinger, 3 = INDI, 4 = Brescianini
        self.stabilizer_estimator   = rospy.get_param("~stabilizer_estimator", 2) # 1 = Complementary filter, 2 = EKF, 3 = unscented Kalman filter
        
        self.test = None
        
        # print(f"Im running with copilot: {self.is_running_with_copilot}")

        # ---------- Crazyflie link ----------
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self._cf = Crazyflie()

        # callbacks
        self._cf.connected     .add_callback(self._connected)
        self._cf.disconnected  .add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._conn_failed)
        self._cf.connection_lost  .add_callback(self._conn_lost)

        uri = uri_helper.uri_from_env(default=id_to_uri(cf_id))
        self._cf.open_link(uri)

        # unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # ---------- ROS pubs/subs ----------
        # note: using absolute names so they live under /cf<ID>/…
        def topic(name): return f"{self.ns}/{name}"

        if self.is_running_with_copilot:
            rospy.Subscriber(topic("copilot_cmd_vel"), Twist, self._twist_cb)
        else:
            rospy.Subscriber(topic("cmd_vel"), Twist, self._twist_cb)

        # pubs (created lazily if log enabled)
        if self.vel_LOG:             self.pub_vel          = rospy.Publisher(topic("crazyflieVel"),           Vector3Stamped, queue_size=10)
        if self.ang_LOG:             self.pub_ang          = rospy.Publisher(topic("crazyflieAng"),           Vector3Stamped, queue_size=10)
        if self.thrust_LOG:          self.pub_thrust       = rospy.Publisher(topic("crazyflieThrust"),        Vector3Stamped, queue_size=10)
        if self.acc_LOG:             self.pub_acc          = rospy.Publisher(topic("crazyflieAcc"),           Vector3Stamped, queue_size=10)
        if self.gyro_raw_LOG:        self.pub_gyro_raw     = rospy.Publisher(topic("crazyflieRawAngRate"),    Vector3Stamped, queue_size=10)
        if self.gyro_LOG:            self.pub_gyro         = rospy.Publisher(topic("crazyflieAngRate"),       Vector3Stamped, queue_size=10)
        if self.stab_rate_LOG:       self.pub_stab_rate    = rospy.Publisher(topic("crazyflieStabRate"),      Float32, queue_size=10)
        if self.control_time_us_LOG: self.control_time_us  = rospy.Publisher(topic("crazyflieControlTimeUs"), Float32, queue_size=10)

        self.pub_is_flying       = rospy.Publisher(topic("crazyflieIsFlying"),   Bool,    queue_size=10)
        self.pub_can_fly         = rospy.Publisher(topic("crazyflieCanFly"),     Bool,    queue_size=10)
        self.pub_z_range         = rospy.Publisher(topic("crazyflieZRange"),     Vector3Stamped, queue_size=10)
        self.pub_battery_voltage = rospy.Publisher(topic("crazyflieBatteryVoltage"), Float32, queue_size=10)
        self.pub_battery_level   = rospy.Publisher(topic("crazyflieBatteryLevel"),   Float32, queue_size=10)
        self.pub_du_r            = rospy.Publisher(topic("crazyflieINDIDuR"),     Float32, queue_size=10)
        self.pub_pos         = rospy.Publisher(topic("crazyfliePos"), Vector3Stamped, queue_size=10)
        self.pub_desired         = rospy.Publisher(topic("crazyflieDesired"), Vector3Stamped, queue_size=10)

        # ---- External pose (OptiTrack/VRPN) ----
        self._have_extpose = False
        self._ext_x = self._ext_y = self._ext_z = 0.0
        self._ext_qx = self._ext_qy = self._ext_qz = 0.0
        self._ext_qw = 1.0

        vrpn_topic = rospy.get_param("~vrpn_pose_topic", f"/vrpn_client_node/cf{cf_id}/pose")
        self._vrpn_sub = rospy.Subscriber(
            vrpn_topic, PoseStamped, self._vrpn_pose_cb, queue_size=1, tcp_nodelay=True
        )

        # send external pose at fixed rate (50–100 Hz is fine; start with 60)
        self._extpose_timer = rospy.Timer(rospy.Duration(1.0/60.0), self._send_extpose)


        # flags to print “log started” once
        self._flags = {}
        self._did_kalman_reset = False

    # ───── connection callbacks ────────────────────────────────────────────────
    def _connected(self, uri):
        rospy.loginfo(f"[cf{self.cf_id}] connected on {uri}")
        rospy.Timer(rospy.Duration(2.0), self._init_cf, oneshot=True)

    def _init_cf(self, _):
        #self._cf.param.get_value("pid_rate.pitch_k_smc", self.test)
        #self._cf.param.set_value("pid_rate.pitch_k_smc", 2.0)
        self._cf.param.set_value("pid_rate.pitch_k_smc", 2.0)
        self.test = self._cf.param.get_value("pid_rate.pitch_k_smc")
        print(f"the value of the pitch k smc is {self.test}")
        # set params
        try:
            self._cf.param.set_value("stabilizer.controller", self.stabilizer_controller)
            self._cf.param.set_value("stabilizer.estimator",  self.stabilizer_estimator)
            self._cf.param.set_value("ctrlINDI.filt_cutoff",   self.filt_cutoff)
            self._cf.param.set_value("ctrlINDI.outerLoopActive",   self.outerLoopActive)
            if self.use_body_rate:
                self._cf.param.set_value("flightmode.stabModeRoll", 0)
                self._cf.param.set_value("flightmode.stabModePitch",0)
                self._cf.param.set_value("flightmode.stabModeYaw",  0)
            else:
                self._cf.param.set_value("flightmode.stabModeRoll", 1)
                self._cf.param.set_value("flightmode.stabModePitch",1)
                self._cf.param.set_value("flightmode.stabModeYaw",  0)
        except Exception as e:
            rospy.logwarn(f"[cf{self.cf_id}] param set failed: {e}")

        # IMPORTANTE para brushless / plataformas com arming manual
        try:
            time.sleep(0.2)
            self._cf.platform.send_arming_request(True)
            rospy.loginfo("Arming request sent")
            time.sleep(0.5)
        except Exception as e:
            rospy.logerr(f"Error sending arming request: {e}")

        self._setup_logs()
        
        # configure logs
        self._setup_logs()

    def _conn_failed(self, uri, msg): rospy.logerr(f"[cf{self.cf_id}] connection failed: {msg}")
    def _conn_lost(self, uri, msg):   rospy.logerr(f"[cf{self.cf_id}] connection lost: {msg}")
    def _disconnected(self, uri):     rospy.loginfo(f"[cf{self.cf_id}] disconnected")

    # ───── logs ────────────────────────────────────────────────────────────────
    def _setup_logs(self):
        # small helper
        def start_log(logconf, data_cb):
            try:
                self._cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(data_cb)
                logconf.error_cb.add_callback(
                    lambda lc, m: rospy.logerr(f"[cf{self.cf_id}] log {lc.name}: {m}")
                )
                logconf.start()
            except KeyError as e:
                rospy.logwarn(f"[cf{self.cf_id}] log {logconf.name}: {e}")

        # supervisor / battery / range are always on
        lg_sup = LogConfig("Supervisor", 1000)
        lg_sup.add_variable("supervisor.info", "uint16_t")
        start_log(lg_sup, self._cb_supervisor)

        lg_bat = LogConfig("Battery", 1000)
        lg_bat.add_variable("pm.vbat", "float")
        lg_bat.add_variable("pm.batteryLevel", "int8_t")
        start_log(lg_bat, self._cb_battery)

        lg_range = LogConfig("Range", 33)
        lg_range.add_variable("range.zrange", "float")
        start_log(lg_range, self._cb_range)

        lg_desired = LogConfig("Desired", 33)
        lg_desired.add_variable("ctrltarget.rollRate", "float")
        lg_desired.add_variable("ctrltarget.pitchRate", "float")
        lg_desired.add_variable("ctrltarget.yaw", "float")
        start_log(lg_desired, self._cb_desired)
        
        #lg_smc = LogConfig("Output SMC", 33)
        #lg_smc.add_variable("pid_rate.roll_outSMC", "float")
        #start_log(lg_smc, self._cb_smc)
        
        if self.control_time_us_LOG:
            lg_control_time = LogConfig("Control Time us", 100)
            lg_control_time.add_variable("timeControl.control_us", "float")
            start_log(lg_control_time, self._cb_lg_control_time)
        
        if self.stab_rate_LOG:
            lg_stabilizer_rate = LogConfig("Stabilizer Rate", 1000)
            lg_stabilizer_rate.add_variable("stabilizer.rtStab", "float")
            start_log(lg_stabilizer_rate, self._cb_stabilizer_rate)

        # optional logs
        if self.pos_LOG:
            lg = LogConfig("Position", 16)
            for p in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
                lg.add_variable(p, "float")
            start_log(lg, self._cb_position)

        if self.vel_LOG:
            lg = LogConfig("Velocity", 16)
            for v in ("stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"):
                lg.add_variable(v, "float")
            start_log(lg, self._cb_velocity)

        if self.ang_LOG:
            lg = LogConfig("Angle", 16)
            for a in ("stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
                lg.add_variable(a, "float")
            start_log(lg, self._cb_angle)

        if self.thrust_LOG:
            lg = LogConfig("Thrust", 16)
            lg.add_variable("stabilizer.thrust", "float")
            start_log(lg, self._cb_thrust)

        if self.acc_LOG:
            lg = LogConfig("Accel", 16)
            for a in ("acc.x", "acc.y", "acc.z"):
                lg.add_variable(a, "float")
            start_log(lg, self._cb_accel)

        if self.gyro_raw_LOG:
            lg = LogConfig("GyroRaw", 16)
            for g in ("gyro.xRaw", "gyro.yRaw", "gyro.zRaw"):
                lg.add_variable(g, "int16_t")
            start_log(lg, self._cb_gyro_raw)

        if self.gyro_LOG:
            lg = LogConfig("Gyro", 16)
            for g in ("gyro.x", "gyro.y", "gyro.z"):
                lg.add_variable(g, "float")
            start_log(lg, self._cb_gyro)

    # ───── log callbacks (publish to ROS) ──────────────────────────────────────
    def _once(self, key, txt):
        if not self._flags.get(key, False):
            rospy.loginfo(f"[cf{self.cf_id}] {txt}")
            self._flags[key] = True

    def _cb_supervisor(self, ts, data, _):
        self._once("sup", "Supervisor log started")
        info = data["supervisor.info"]
        self.pub_can_fly.publish(bool((info >> 3) & 1))
        self.pub_is_flying.publish(bool((info >> 4) & 1))

    def _cb_battery(self, ts, data, _):
        self._once("bat", "Battery log started")
        self.pub_battery_voltage.publish(Float32(data["pm.vbat"]))
        self.pub_battery_level.publish(Float32(float(data["pm.batteryLevel"])))

    def _cb_range(self, ts, data, _):
        self._once("range", "Range log started")
        z = data["range.zrange"] / 1000.0  # mm→m
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.z = z
        self.pub_z_range.publish(msg)
        
    def _cb_smc(self, ts, data, _):
        self._once("out_smc", "Out SMC log started")
        out_smc = data["pid_rate.roll_outSMC"]
        
        #print(out_smc)
        #msg = Vector3Stamped()
        #msg.header.stamp = rospy.Time.now()
        #msg.vector.z = z
        #self.pub_z_range.publish(msg)

    def _cb_desired(self, ts, data, _):
        self._once("pos", "Position log started")
        # estimated position
        pos = Vector3Stamped()
        pos.header.stamp = rospy.Time.now()
        pos.vector.x = data["ctrltarget.rollRate"]
        pos.vector.y = data["ctrltarget.pitchRate"]
        pos.vector.z = data["ctrltarget.yaw"]
        self.pub_desired.publish(pos)
    
    def _cb_stabilizer_rate(self, ts, data, _):
        self._once("stabilizer_rate", "Stabilizer Rate log started")
        stabilizer_rate = data["stabilizer.rtStab"]
        
        self.pub_stab_rate.publish(stabilizer_rate)
        
    def _cb_lg_control_time(self, ts, data, _):
        self._once("control_time", "Control Time us log started")
        control_time = data["timeControl.control_us"]
        self.control_time_us.publish(control_time)

    def _cb_position(self, ts, data, _):
        self._once("pos", "Position log started")
        # estimated position
        pos = Vector3Stamped()
        pos.header.stamp = rospy.Time.now()
        pos.vector.x = data["stateEstimate.x"]
        pos.vector.y = data["stateEstimate.y"]
        pos.vector.z = data["stateEstimate.z"]
        self.pub_pos.publish(pos)

    def _cb_velocity(self, ts, data, _):
        self._once("vel", "Velocity log started")
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = data["stateEstimate.vx"]
        msg.vector.y = data["stateEstimate.vy"]
        msg.vector.z = data["stateEstimate.vz"]
        self.pub_vel.publish(msg)

    def _cb_angle(self, ts, data, _):
        self._once("ang", "Angle log started")
        deg2rad = 0.01745329252
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = data["stateEstimate.roll"]  * deg2rad
        msg.vector.y = -data["stateEstimate.pitch"]* deg2rad
        msg.vector.z = data["stateEstimate.yaw"]   * deg2rad
        self.pub_ang.publish(msg)

    def _cb_thrust(self, ts, data, _):
        self._once("thr", "Thrust log started")
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.z = data["stabilizer.thrust"]
        self.pub_thrust.publish(msg)

    def _cb_accel(self, ts, data, _):
        self._once("acc", "Acceleration log started")
        g = 9.81
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = -data["acc.x"] * g
        msg.vector.y = -data["acc.y"] * g
        msg.vector.z = -data["acc.z"] * g
        self.pub_acc.publish(msg)

    def _cb_gyro_raw(self, ts, data, _):
        self._once("gyro_raw", "Raw Gyro log started")
        scale = 0.001065
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = data["gyro.xRaw"] * scale
        msg.vector.y = data["gyro.yRaw"] * scale
        msg.vector.z = data["gyro.zRaw"] * scale
        self.pub_gyro_raw.publish(msg)

    def _cb_gyro(self, ts, data, _):
        self._once("gyro", "Gyro log started")
        deg2rad = 0.01745329252
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = data["gyro.x"] * deg2rad
        msg.vector.y = data["gyro.y"] * deg2rad
        msg.vector.z = data["gyro.z"] * deg2rad
        self.pub_gyro.publish(msg)

    # ───── cmd_vel subscriber ─────────────────────────────────────────────────
    def _twist_cb(self, msg: Twist):
        if self.use_body_rate:
            rollrate  =  msg.angular.x * 57.2958
            pitchrate =  msg.angular.y * 57.2958
            yawrate   = -msg.angular.z * 57.2958
            thrust    =  int(msg.linear.z)
            self._cf.commander.send_setpoint(rollrate, pitchrate, yawrate, thrust)
        else:
            roll   =  msg.angular.x * 57.2958
            pitch  =  msg.angular.y * 57.2958
            yawrate= -msg.angular.z * 57.2958
            thrust =  int(msg.linear.z)
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def _vrpn_pose_cb(self, msg: PoseStamped):
        
        if not self._did_kalman_reset:
            self._did_kalman_reset = True
            rospy.Timer(rospy.Duration(0.2), self._kalman_reset_oneshot, oneshot=True)

        p = msg.pose.position
        q = msg.pose.orientation

        self._ext_x, self._ext_y, self._ext_z = p.x, p.y, p.z
        self._ext_qx, self._ext_qy, self._ext_qz, self._ext_qw = q.x, q.y, q.z, q.w
        self._have_extpose = True
    
    def _send_extpose(self, event):
        if not self._have_extpose:
            return
        else:
            return

        qx, qy, qz, qw = self._ext_qx, self._ext_qy, self._ext_qz, self._ext_qw
        n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n < 1e-9:
            return
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

        # This is the *only* line that matters for “using OptiTrack”
        self._cf.extpos.send_extpose(self._ext_x, self._ext_y, self._ext_z, qx, qy, qz, qw)
        # If you want position-only:
        # self._cf.extpos.send_extpos(self._ext_x, self._ext_y, self._ext_z)

        # rospy.loginfo_throttle(1.0, f"extpose -> cf: {self._ext_x:.2f} {self._ext_y:.2f} {self._ext_z:.2f}")
    
    def _kalman_reset_oneshot(self, _):
        try:
            self._cf.param.set_value("locSrv.extPosStdDev", 0.002)
            self._cf.param.set_value("locSrv.extQuatStdDev", 0.02)

            self._cf.param.set_value("kalman.resetEstimation", 1)
            rospy.sleep(0.1)
            self._cf.param.set_value("kalman.resetEstimation", 0)
            rospy.loginfo(f"[cf{self.cf_id}] kalman reset done (after first VRPN)")
        except Exception as e:
            rospy.logwarn(f"[cf{self.cf_id}] kalman reset failed: {e}")

    # ───── cleanup ────────────────────────────────────────────────────────────
    def close(self):
        try:
            self._cf.close_link()
        except Exception:
            pass

def main():
    rospy.init_node("multi_crazyflie_server")

    raw = rospy.get_param("~ids", [1])
    if isinstance(raw, str):
        raw = raw.strip("[]")
        ids = [int(x) for x in raw.replace(',', ' ').split()]
    else:
        ids = list(raw)
    if not ids:
        rospy.logfatal("~ids is empty")
        sys.exit(1)

    servers = [CrazyflieServer(cf_id=i) for i in ids]

    # ---------- graceful shutdown ------------------------------------------
    def _close_everything():
        rospy.loginfo("Shutting down links...")
        for s in servers:
            s.close()           # closes USB / radio, signals cflib threads
        time.sleep(0.2)          # give cflib worker threads time to exit

    rospy.on_shutdown(_close_everything)

    # Catch Ctrl-C ourselves and forward it to rospy
    signal.signal(signal.SIGINT, lambda *_: rospy.signal_shutdown("SIGINT"))

    # -----------------------------------------------------------------------
    rospy.spin()                 # returns when signal_shutdown() called
    _close_everything()          # second call is harmless
    sys.exit(0)

if __name__ == "__main__":
    main()
