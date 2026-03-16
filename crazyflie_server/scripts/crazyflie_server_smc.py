#!/usr/bin/env python3

import time
import sys
import signal
import math

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, PoseStamped, Vector3, Pose
from std_msgs.msg import Bool, Float32

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import struct
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

# ---- custom generic setpoint type (must match firmware enum accType) ----

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
        self.use_body_rate          = rospy.get_param("~use_body_rate", False)
        self.filt_cutoff            = rospy.get_param("~filt_cutoff", 10)
        self.outerLoopActive        = rospy.get_param("~outerLoopActive", False)
        self.pos_LOG                = rospy.get_param("~pos_LOG", False)
        self.vel_LOG                = rospy.get_param("~vel_LOG", False)
        self.ang_LOG                = rospy.get_param("~ang_LOG", False)
        self.thrust_LOG             = rospy.get_param("~thrust_LOG", False)
        self.acc_LOG                = rospy.get_param("~acc_LOG", False)
        self.gyro_raw_LOG           = rospy.get_param("~gyro_raw_LOG", False)
        self.gyro_LOG               = rospy.get_param("~gyro_LOG", False)
        self.stab_rate_LOG          = rospy.get_param("~stab_rate_LOG", False)
        self.control_time_us_LOG    = rospy.get_param("~control_time_us_LOG", False)
        self.is_running_with_copilot= rospy.get_param("~is_running_with_copilot", False)
        self.stabilizer_controller  = rospy.get_param("~stabilizer_controller", 1) # 1 = PID, 2 = Mellinger, 3 = INDI, 4 = Brescianini
        self.stabilizer_estimator   = rospy.get_param("~stabilizer_estimator", 2) # 1 = Complementary filter, 2 = EKF, 3 = unscented Kalman filter

        # Parâmetros SMC
        self.use_smc                = rospy.get_param("~use_smc", 0)
        self.is_PD_ASMC             = rospy.get_param("~is_PD_ASMC", 0)
        self.smc_mode               = rospy.get_param("~smc_mode", 0) # -1 = Desligado, 0 = Asmc, 1 = Super Twist, 2 = Roy SMC

        self.k_smc_roll             = rospy.get_param("~k_smc_roll", 1)
        self.k_smc_pitch            = rospy.get_param("~k_smc_pitch", 1)
        self.k_smc_yaw              = rospy.get_param("~k_smc_yaw", 1)

        self.lambda_smc_roll        = rospy.get_param("~lambda_smc_roll", 1)
        self.lambda_smc_pitch       = rospy.get_param("~lambda_smc_pitch", 1)
        self.lambda_smc_yaw         = rospy.get_param("~lambda_smc_yaw", 1)

        self.sigma_smc_roll         = rospy.get_param("~sigma_smc_roll", 1)
        self.sigma_smc_pitch        = rospy.get_param("~sigma_smc_pitch", 1)
        self.sigma_smc_yaw          = rospy.get_param("~sigma_smc_yaw", 1)

        self.ki_smc_roll             = rospy.get_param("~ki_smc_roll", 1)
        self.ki_smc_pitch            = rospy.get_param("~ki_smc_pitch", 1)
        self.ki_smc_yaw              = rospy.get_param("~ki_smc_yaw", 1)

        self.delta_smc_roll             = rospy.get_param("~delta_smc_roll", 1)
        self.delta_smc_pitch            = rospy.get_param("~delta_smc_pitch", 1)
        self.delta_smc_yaw              = rospy.get_param("~delta_smc_yaw", 1)

        self.kj_smc_roll             = rospy.get_param("~kj_smc_roll", 1)
        self.kj_smc_pitch            = rospy.get_param("~kj_smc_pitch", 1)
        self.kj_smc_yaw              = rospy.get_param("~kj_smc_yaw", 1)

        self.sigma_sdelta_smc_roll         = rospy.get_param("~sigma_sdelta_smc_roll", 1)
        self.sigma_sdelta_smc_pitch        = rospy.get_param("~sigma_sdelta_smc_pitch", 1)
        self.sigma_sdelta_smc_yaw          = rospy.get_param("~sigma_sdelta_smc_yaw", 1)

        self.k_min_smc             = rospy.get_param("~k_min_smc", 1)
        self.k_max_smc             = rospy.get_param("~k_max_smc", 1)

        # Super Twist 
        self.i_st_limit_smc             = rospy.get_param("~i_st_limit_smc", 1)

        self.ki_smc_st_roll             = rospy.get_param("~ki_smc_st_roll", 1)
        self.ki_smc_st_pitch            = rospy.get_param("~ki_smc_st_pitch", 1)
        self.ki_smc_st_yaw              = rospy.get_param("~ki_smc_st_yaw", 1)

        self.kj_smc_st_roll             = rospy.get_param("~kj_smc_st_roll", 1)
        self.kj_smc_st_pitch            = rospy.get_param("~kj_smc_st_pitch", 1)
        self.kj_smc_st_yaw              = rospy.get_param("~kj_smc_st_yaw", 1)

        self.k_phi = rospy.get_param("~k_phi", None)
        self.k_theta = rospy.get_param("~k_theta", None)
        self.k_psi = rospy.get_param("~k_psi", None)

        self.sat_roll = rospy.get_param("~sat_roll", None)
        self.sat_pitch = rospy.get_param("~sat_pitch", None)
        self.sat_yaw = rospy.get_param("~sat_yaw", None)

        # Roy smc
        self.alpha0              = rospy.get_param("~alpha0", 1)
        self.alpha1              = rospy.get_param("~alpha1", 1)

        self.h0              = rospy.get_param("~h0", 1)
        self.h1              = rospy.get_param("~h1", 1)

        self.ax = 0
        self.ay = 0
        self.az = 0
        # print(f"Im running with copilot: {self.is_running_with_copilot}")

        # ---------- Crazyflie link ----------
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self._cf = Crazyflie()
        uri = uri_helper.uri_from_env(default=id_to_uri(cf_id))
        self._cf.open_link(uri)

        # callbacks
        self._cf.connected     .add_callback(self._connected)
        self._cf.disconnected  .add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._conn_failed)
        self._cf.connection_lost  .add_callback(self._conn_lost)

        # unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # ---------- ROS pubs/subs ----------
        # note: using absolute names so they live under /cf<ID>/…
        def topic(name): return f"{self.ns}/{name}"

        if self.is_running_with_copilot:
            rospy.Subscriber(topic("copilot_cmd_vel"), Twist, self._twist_cb)
        else:
            rospy.Subscriber(topic("cmd_vel"), Twist, self._twist_cb)

        rospy.Subscriber(topic("cmd_vel_smc"), Pose, self._twist_cb_smc)
        
        # pubs (created lazily if log enabled)
        if self.vel_LOG:             self.pub_vel          = rospy.Publisher(topic("crazyflieVel"),           Vector3Stamped, queue_size=10)
        if self.ang_LOG:             self.pub_ang          = rospy.Publisher(topic("crazyflieAng"),           Vector3Stamped, queue_size=10)
        if self.thrust_LOG:          self.pub_thrust       = rospy.Publisher(topic("crazyflieThrust"),        Vector3Stamped, queue_size=10)
        if self.acc_LOG:             self.pub_acc          = rospy.Publisher(topic("crazyflieAcc"),           Vector3Stamped, queue_size=10)
        if self.gyro_raw_LOG:        self.pub_gyro_raw     = rospy.Publisher(topic("crazyflieRawAngRate"),    Vector3Stamped, queue_size=10)
        if self.gyro_LOG:            self.pub_gyro         = rospy.Publisher(topic("crazyflieAngRate"),       Vector3Stamped, queue_size=10)
        if self.stab_rate_LOG:       self.pub_stab_rate    = rospy.Publisher(topic("crazyflieStabRate"),      Float32, queue_size=10)
        if self.control_time_us_LOG: self.control_time_us  = rospy.Publisher(topic("crazyflieControlTimeUs"), Float32, queue_size=10)

        
        # self.pub_ang_filt        = rospy.Publisher(topic("crazyflieAngFilt"),         Vector3Stamped,    queue_size=10)

        self.pub_is_flying       = rospy.Publisher(topic("crazyflieIsFlying"),        Bool,    queue_size=10)
        self.pub_can_fly         = rospy.Publisher(topic("crazyflieCanFly"),          Bool,    queue_size=10)
        # self.pub_z_range         = rospy.Publisher(topic("crazyflieZRange"),         Vector3Stamped, queue_size=10)
        self.pub_battery_voltage = rospy.Publisher(topic("crazyflieBatteryVoltage"),  Float32, queue_size=10)
        #self.pub_battery_level   = rospy.Publisher(topic("crazyflieBatteryLevel"),   Float32, queue_size=10)
        #self.pub_pos             = rospy.Publisher(topic("crazyfliePos"),            Vector3Stamped, queue_size=10)
        #self.pub_desired         = rospy.Publisher(topic("crazyflieDesired"),        Vector3Stamped, queue_size=10)
        self.pub_smc_out         = rospy.Publisher(topic("crazyflieSMCOut"),          Vector3Stamped, queue_size=10)
        # self.pub_pid_out         = rospy.Publisher(topic("crazyfliePIDOut"),          Vector3Stamped, queue_size=10)
        self.pub_pid_out_pitch   = rospy.Publisher(topic("crazyfliePIDOutPitch"),     Vector3Stamped, queue_size=10)
        self.pub_pid_out_roll    = rospy.Publisher(topic("crazyfliePIDOutRoll"),     Vector3Stamped, queue_size=10)
        self.pub_s_smc           = rospy.Publisher(topic("crazyflieSSMC"),            Vector3Stamped, queue_size=10)
        # self.pub_k_smc           = rospy.Publisher(topic("crazyflieKSMC"),            Vector3Stamped, queue_size=10)
        #self.pub_er_smc          = rospy.Publisher(topic("crazyflieErSMC"),           Vector3Stamped, queue_size=10)
        # self.pub_r_error         = rospy.Publisher(topic("crazyflieRError"),          Vector3Stamped, queue_size=10)
        self.pub_integ_st        = rospy.Publisher(topic("crazyflieIntegST"),         Vector3Stamped, queue_size=10)
        # self.pub_h0        = rospy.Publisher(topic("crazyflieH0"),         Vector3Stamped, queue_size=10)
        # self.pub_h1        = rospy.Publisher(topic("crazyflieH1"),         Vector3Stamped, queue_size=10)
        
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
        self._extpose_timer = rospy.Timer(rospy.Duration(1.0/100.0), self._send_extpose)

        # flags to print “log started” once
        self._flags = {}
        self._did_kalman_reset = False

    # ───── connection callbacks ────────────────────────────────────────────────
    def _connected(self, uri):
        rospy.loginfo(f"[cf{self.cf_id}] connected on {uri}")
        rospy.Timer(rospy.Duration(2.0), self._init_cf, oneshot=True)

    def _init_cf(self, _):
        if self.use_smc:
            self._cf.param.set_value("smc.useSMC", 1)
        else:
            self._cf.param.set_value("smc.useSMC", 0)

        if self.is_PD_ASMC:
            self._cf.param.set_value("smc.is_PD_ASMC", 1)
        else:
            self._cf.param.set_value("smc.is_PD_ASMC", 0)

        self._cf.param.set_value("smc.smc_mode", self.smc_mode)      

        self._cf.param.set_value("smc_rate.roll_alpha0_smc", self.alpha0)
        self._cf.param.set_value("smc_rate.pitch_alpha0_smc", self.alpha0)

        self._cf.param.set_value("smc_rate.roll_alpha1_smc", self.alpha1)
        self._cf.param.set_value("smc_rate.pitch_alpha1_smc", self.alpha1)

        self._cf.param.set_value("smc_rate.roll_h0_smc", self.h0)
        self._cf.param.set_value("smc_rate.pitch_h0_smc", self.h0)

        self._cf.param.set_value("smc_rate.roll_h1_smc", self.h1)
        self._cf.param.set_value("smc_rate.pitch_h1_smc", self.h1)
        
        self._cf.param.set_value("pid_rate.roll_k_smc", self.k_smc_roll)
        self._cf.param.set_value("pid_rate.pitch_k_smc", self.k_smc_pitch)
        self._cf.param.set_value("pid_rate.yaw_k_smc", self.k_smc_yaw)

        self._cf.param.set_value("pid_rate.roll_lambda_smc", self.lambda_smc_roll)
        self._cf.param.set_value("pid_rate.pitch_lambda_smc", self.lambda_smc_pitch)
        self._cf.param.set_value("pid_rate.yaw_lambda_smc", self.lambda_smc_yaw)

        self._cf.param.set_value("pid_rate.roll_sigma_smc", self.sigma_smc_roll)
        self._cf.param.set_value("pid_rate.pitch_sigma_smc", self.sigma_smc_pitch)
        self._cf.param.set_value("pid_rate.yaw_sigma_smc", self.sigma_smc_yaw)

        self._cf.param.set_value("pid_rate.roll_delta_smc", self.delta_smc_roll)
        self._cf.param.set_value("pid_rate.pitch_delta_smc", self.delta_smc_pitch)
        self._cf.param.set_value("pid_rate.yaw_delta_smc", self.delta_smc_yaw)

        if self.smc_mode:
            self._cf.param.set_value("pid_rate.roll_ki_smc", self.ki_smc_st_roll)
            self._cf.param.set_value("pid_rate.pitch_ki_smc", self.ki_smc_st_pitch)
            self._cf.param.set_value("pid_rate.yaw_ki_smc", self.ki_smc_st_yaw)

            self._cf.param.set_value("pid_rate.roll_kj_smc", self.kj_smc_st_roll)
            self._cf.param.set_value("pid_rate.pitch_kj_smc", self.kj_smc_st_pitch)
            self._cf.param.set_value("pid_rate.yaw_kj_smc", self.kj_smc_st_yaw)
        else:
            self._cf.param.set_value("pid_rate.roll_ki_smc", self.ki_smc_roll)
            self._cf.param.set_value("pid_rate.pitch_ki_smc", self.ki_smc_pitch)
            self._cf.param.set_value("pid_rate.yaw_ki_smc", self.ki_smc_yaw)

            self._cf.param.set_value("pid_rate.roll_kj_smc", self.kj_smc_roll)
            self._cf.param.set_value("pid_rate.pitch_kj_smc", self.kj_smc_pitch)
            self._cf.param.set_value("pid_rate.yaw_kj_smc", self.kj_smc_yaw)

            

        self._cf.param.set_value("pid_rate.roll_sdelta", self.sigma_sdelta_smc_roll)
        self._cf.param.set_value("pid_rate.pitch_sdelta", self.sigma_sdelta_smc_pitch)
        self._cf.param.set_value("pid_rate.yaw_sdelta", self.sigma_sdelta_smc_yaw)
        
        self._cf.param.set_value("pid_rate.roll_k_min_smc", self.k_min_smc)
        self._cf.param.set_value("pid_rate.roll_k_max_smc", self.k_max_smc)

        self._cf.param.set_value("pid_rate.pitch_k_min_smc", self.k_min_smc)
        self._cf.param.set_value("pid_rate.pitch_k_max_smc", self.k_max_smc)

        self._cf.param.set_value("pid_rate.yaw_k_min_smc", self.k_min_smc)
        self._cf.param.set_value("pid_rate.yaw_k_max_smc", self.k_max_smc)

        self._cf.param.set_value("pid_rate.roll_i_lim_st", self.i_st_limit_smc)

        self._cf.param.set_value("pid_rate.pitch_i_lim_st", self.i_st_limit_smc)

        self._cf.param.set_value("pid_rate.yaw_i_lim_st", self.i_st_limit_smc)


        if self.k_phi is not None:
            self._cf.param.set_value("smc.k_phi", float(self.k_phi))
            #print(self.k_phi)
            #print(f"The value of the K_phi is: {self._cf.param.get_value('smc.k_phi')}")
        if self.k_theta is not None:
            self._cf.param.set_value("smc.k_theta", float(self.k_theta))
            #print(f"The value of the k_theta is: {self._cf.param.get_value('smc.k_theta')}")
        if self.k_psi is not None:
            self._cf.param.set_value("smc.k_psi", float(self.k_psi))
            #print(f"The value of the k_psi is: {self._cf.param.get_value('smc.k_psi')}")
        
        if self.sat_roll is not None:
            self._cf.param.set_value("smc.sat_roll", float(self.sat_roll))
            print(f"The value of the sat_roll is: {self._cf.param.get_value('smc.sat_roll')}")
        if self.sat_pitch is not None:
            self._cf.param.set_value("smc.sat_pitch", float(self.sat_pitch))
        if self.sat_yaw is not None:
            self._cf.param.set_value("smc.sat_yaw", float(self.sat_yaw))

        try:
            self._cf.param.set_value("stabilizer.controller", self.stabilizer_controller)
            self._cf.param.set_value("stabilizer.estimator",  self.stabilizer_estimator)
            self._cf.param.set_value("ctrlINDI.filt_cutoff",   self.filt_cutoff)
            self._cf.param.set_value("ctrlINDI.outerLoopActive",   self.outerLoopActive)
            if self.use_body_rate:
                self._cf.param.set_value("flightmode.stabModeRoll", 0)
                self._cf.param.set_value("flightmode.stabModePitch",0)
                self._cf.param.set_value("flightmode.stabModeYaw",  0)
                print("Sending attitude rate setpoints.")
            else:
                self._cf.param.set_value("flightmode.stabModeRoll", 1)
                self._cf.param.set_value("flightmode.stabModePitch",1)
                self._cf.param.set_value("flightmode.stabModeYaw",  0)
                print("Sending attitude setpoints.")
        except Exception as e:
            rospy.logwarn(f"[cf{self.cf_id}] param set failed: {e}")

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

        # lg_range = LogConfig("Range", 33)
        # lg_range.add_variable("range.zrange", "float")
        # start_log(lg_range, self._cb_range)
        
        lg_smc_out = LogConfig("Output SMC", 33)
        lg_smc_out.add_variable("pid_rate.roll_outSMC", "float")
        lg_smc_out.add_variable("pid_rate.pitch_outSMC", "float")
        lg_smc_out.add_variable("pid_rate.yaw_outSMC", "float")
        start_log(lg_smc_out, self._cb_smc_out)

        # lg_pid_out = LogConfig("Output PID", 33)
        # lg_pid_out.add_variable("pid_rate.roll_outPID", "float")
        # lg_pid_out.add_variable("pid_rate.pitch_outPID", "float")
        # lg_pid_out.add_variable("pid_rate.yaw_outPID", "float")
        # start_log(lg_pid_out, self._cb_pid_out)

        lg_s_smc = LogConfig("s SMC", 33)
        lg_s_smc.add_variable("pid_rate.roll_s_smc", "float")
        lg_s_smc.add_variable("pid_rate.pitch_s_smc", "float")
        lg_s_smc.add_variable("pid_rate.yaw_s_smc", "float")
        start_log(lg_s_smc, self._cb_s_smc)

        lg_pid_out_roll = LogConfig("Out PID separate", 66)
        lg_pid_out_roll.add_variable("pid_rate.roll_outP", "float")
        lg_pid_out_roll.add_variable("pid_rate.roll_outI", "float")
        lg_pid_out_roll.add_variable("pid_rate.roll_outD", "float")
        start_log(lg_pid_out_roll, self._cb_pid_out_roll)

        lg_pid_out_pitch = LogConfig("Out PID separate", 66)
        lg_pid_out_pitch.add_variable("pid_rate.pitch_outP", "float")
        lg_pid_out_pitch.add_variable("pid_rate.pitch_outI", "float")
        lg_pid_out_pitch.add_variable("pid_rate.pitch_outD", "float")
        start_log(lg_pid_out_pitch, self._cb_pid_out_pitch)

        # lg_pid_out_pitch = LogConfig("K SMC", 33)
        # lg_pid_out_pitch.add_variable("pid_rate.roll_k_smc", "float")
        # lg_pid_out_pitch.add_variable("pid_rate.pitch_k_smc", "float")
        # lg_pid_out_pitch.add_variable("pid_rate.yaw_k_smc", "float")
        # start_log(lg_pid_out_pitch, self._cb_k_smc)

        # lg_e_error = LogConfig("R error", 33)
        # lg_e_error.add_variable("smc.R13_error", "float")
        # lg_e_error.add_variable("smc.R23_error", "float")
        # start_log(lg_e_error, self._r_error_smc)

        # lg_er_smc = LogConfig("Er SMC", 33)
        # lg_er_smc.add_variable("smc.er_phi", "float")
        # lg_er_smc.add_variable("smc.er_theta", "float")
        # lg_er_smc.add_variable("smc.er_psi", "float")
        # start_log(lg_er_smc, self._cb_er_smc)

        # lg_desired = LogConfig("Desired", 33)
        # lg_desired.add_variable("controller.rollRate", "float")
        # lg_desired.add_variable("controller.pitchRate", "float")
        # lg_desired.add_variable("controller.yawRate", "float")
        # start_log(lg_desired, self._cb_desired)

        lg_integ_st = LogConfig("integ_Super_twist", 33)
        lg_integ_st.add_variable("pid_rate.roll_integ_st", "float")
        lg_integ_st.add_variable("pid_rate.pitch_integ_st", "float")
        start_log(lg_integ_st, self._integ_st)

        # lg_h0_h1 = LogConfig("h0", 33)
        # lg_h0_h1.add_variable("smc_rate.roll_h0_smc", "float")
        # lg_h0_h1.add_variable("smc_rate.pitch_h0_smc", "float")
        # lg_h0_h1.add_variable("smc_rate.roll_h1_smc", "float")
        # lg_h0_h1.add_variable("smc_rate.pitch_h1_smc", "float")
        # start_log(lg_h0_h1, self._h0_h1)
        
        if self.control_time_us_LOG:
            lg_control_time = LogConfig("Control Time us", 100)
            lg_control_time.add_variable("timeControl.control_us", "float")
            start_log(lg_control_time, self._cb_lg_z_time)
        
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

            # lg_filt = LogConfig("Angle filt", 16)
            # for a in ("smc.phi_filt", "smc.theta_filt", "smc.psi_filt"):
            #     lg_filt.add_variable(a, "float")
            # start_log(lg_filt, self._cb_angle_filt)

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

        print(f"The value of the K_phi is: {self._cf.param.get_value('smc.k_phi')}")

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
        
    def _cb_smc_out(self, ts, data, _):
        self._once("out_smc", "Out SMC log started")

        smc_out = Vector3Stamped()
        smc_out.header.stamp = rospy.Time.now()
        smc_out.vector.x = data["pid_rate.roll_outSMC"]
        smc_out.vector.y = data["pid_rate.pitch_outSMC"]
        smc_out.vector.z = data["pid_rate.yaw_outSMC"]
        self.pub_smc_out.publish(smc_out)

    def _cb_pid_out(self, ts, data, _):
        self._once("out_pid", "Out PID log started")

        pid_out = Vector3Stamped()
        pid_out.header.stamp = rospy.Time.now()
        pid_out.vector.x = data["pid_rate.roll_outPID"]
        pid_out.vector.y = data["pid_rate.pitch_outPID"]
        pid_out.vector.z = data["pid_rate.yaw_outPID"]
        self.pub_pid_out.publish(pid_out)

    def _cb_pid_out_pitch(self, ts, data, _):
        self._once("out_pid_pitch", "Out PID pitch log started")

        pid_out = Vector3Stamped()
        pid_out.header.stamp = rospy.Time.now()
        pid_out.vector.x = data["pid_rate.pitch_outP"]
        pid_out.vector.y = data["pid_rate.pitch_outI"]
        pid_out.vector.z = data["pid_rate.pitch_outD"]
        self.pub_pid_out_pitch.publish(pid_out)

    def _cb_pid_out_roll(self, ts, data, _):
        self._once("out_pid_roll", "Out PID roll log started")

        pid_out = Vector3Stamped()
        pid_out.header.stamp = rospy.Time.now()
        pid_out.vector.x = data["pid_rate.roll_outP"]
        pid_out.vector.y = data["pid_rate.roll_outI"]
        pid_out.vector.z = data["pid_rate.roll_outD"]
        self.pub_pid_out_roll.publish(pid_out)

    def _cb_k_smc(self, ts, data, _):
        self._once("k_smc", "K smc log started")

        pid_out = Vector3Stamped()
        pid_out.header.stamp = rospy.Time.now()
        pid_out.vector.x = data["pid_rate.roll_k_smc"]
        pid_out.vector.y = data["pid_rate.pitch_k_smc"]
        pid_out.vector.z = data["pid_rate.yaw_k_smc"]
        self.pub_k_smc.publish(pid_out)

    def _r_error_smc(self, ts, data, _):
        self._once("r_error_smc", "R error log started")

        r_error_smc = Vector3Stamped()
        r_error_smc.header.stamp = rospy.Time.now()
        r_error_smc.vector.x = data["smc.R13_error"]
        r_error_smc.vector.y = data["smc.R23_error"]
        self.pub_r_error.publish(r_error_smc)

    def _integ_st(self, ts, data, _):
        self._once("test thrust", "Integ Super Twist log started")

        integ_st = Vector3Stamped()
        integ_st.header.stamp = rospy.Time.now()
        integ_st.vector.x = data["pid_rate.roll_integ_st"]*self.kj_smc_st_roll
        integ_st.vector.y = data["pid_rate.pitch_integ_st"]*self.kj_smc_st_pitch
        self.pub_integ_st.publish(integ_st)

    def _h0_h1(self, ts, data, _):
        self._once("h0 h1", "h0 h1 log started")

        h0 = Vector3Stamped()
        h0.header.stamp = rospy.Time.now()
        h0.vector.x = data["smc_rate.roll_h0_smc"]
        h0.vector.y = data["smc_rate.pitch_h0_smc"]
        self.pub_h0.publish(h0)

        h1 = Vector3Stamped()
        h1.header.stamp = rospy.Time.now()
        h1.vector.x = data["smc_rate.roll_h1_smc"]
        h1.vector.y = data["smc_rate.pitch_h1_smc"]
        self.pub_h1.publish(h1)
    
    def _cb_s_smc(self, ts, data, _):
        self._once("s_smc", "Non saturated s SMC log started")

        s_smc = Vector3Stamped()
        s_smc.header.stamp = rospy.Time.now()
        s_smc.vector.x = data["pid_rate.roll_s_smc"]
        s_smc.vector.y = data["pid_rate.pitch_s_smc"]
        s_smc.vector.z = data["pid_rate.yaw_s_smc"]
        self.pub_s_smc.publish(s_smc)

    def _cb_er_smc(self, ts, data, _):
        self._once("er_smc", "Er SMC log started")

        s_smc = Vector3Stamped()
        s_smc.header.stamp = rospy.Time.now()
        s_smc.vector.x = data["smc.er_phi"]
        s_smc.vector.y = data["smc.er_theta"]
        s_smc.vector.z = data["smc.er_psi"]
        self.pub_er_smc.publish(s_smc)

    def _cb_desired(self, ts, data, _):
        self._once("pos", "Position log started")
        # estimated position
        pos = Vector3Stamped()
        pos.header.stamp = rospy.Time.now()
        pos.vector.x = data["controller.rollRate"]
        pos.vector.y = data["controller.pitchRate"]
        pos.vector.z = data["controller.yawRate"]
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

    def _cb_angle_filt(self, ts, data, _):
        self._once("ang_filt", "Angle log started")
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = data["smc.phi_filt"]
        msg.vector.y = -data["smc.theta_filt"]
        msg.vector.z = data["smc.psi_filt"]
        self.pub_ang_filt.publish(msg)
    
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

            #self._send_acc_setpoint(rollrate, pitchrate, yawrate, thrust, self.ax, self.ay, self.az)
        else:
            roll   =  msg.angular.x * 57.2958
            pitch  =  msg.angular.y * 57.2958
            yawrate= -msg.angular.z * 57.2958
            thrust =  int(msg.linear.z)
            #self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    
    def _twist_cb_smc(self, msg: Pose):
        ax   =  msg.position.x
        ay   =  msg.position.y
        az   =  msg.position.z

        rollrate  =  msg.orientation.x * 57.2958
        pitchrate =  msg.orientation.y * 57.2958
        yawrate   = -msg.orientation.z * 57.2958
        thrust    =  int(msg.orientation.w)

        #print(thrust)

        self._send_acc_setpoint(rollrate, pitchrate, yawrate, thrust, ax, ay, az) 

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
        # else:
        #     return

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
    
    TYPE_ACC = 12
    MIN_THRUST = 1000
    MAX_THRUST = 60000
    def _send_acc_setpoint(self, roll_deg, pitch_deg, yaw_deg, thrust_u16, ax, ay, az):
        """
        Firmware struct (packed):
          float roll, pitch, yaw; uint16 thrust; float ax, ay, az
        Packet on the wire:pitchRate
          [type:uint8][roll f32][pitch f32][yaw f32][thrust u16][ax f32][ay f32][az f32]
        Total payload = 1 + 4*3 + 2 + 4*3 = 27 bytes
        """
        thrust_u16 = int(max(0, min(thrust_u16, 65535)))

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.channel = 0  # SET_SETPOINT_CHANNEL is 0

        pk.data = struct.pack(
            '<BfffHfff',
            self.TYPE_ACC,
            float(roll_deg), float(-pitch_deg), float(yaw_deg),
            thrust_u16,
            float(ax), float(ay), float(az)
        )
        self._cf.send_packet(pk)

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
