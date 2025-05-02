/*********************************************************************
 *  hover_controller_takeoff.cpp  –  Pattern-A (Timer + MT-Spinner)
 *********************************************************************/

 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>
 #include <geometry_msgs/Vector3Stamped.h>
 #include <std_msgs/Bool.h>
 #include <std_srvs/Trigger.h>
 #include <cmath>
 #include <algorithm>
 #include <atomic>
 #include <mutex>
 
 enum class Mode { LANDED, IDLE, TAKEOFF, HOVER, LANDING, STOPPED_FLYING };
 
 class HoverController
 {
 public:
   explicit HoverController(ros::NodeHandle& nh)
   {
     /* ─── Parameters ─────────────────────────────────────── */
     nh.param("idle_time",            idle_time_,        2.0);
     nh.param("idle_pwm",             idle_pwm_,     12000.0);
     nh.param("takeoff_duration",     takeoff_dur_,      2.0);
     nh.param("takeoff_altitude",     hover_alt_,        1.0);
     nh.param("landing_velocity",     land_vel_,        0.3);
     nh.param("controller_frequency", controller_freq_, 80.0);
 
     nh.param("kp_z",  Kp_z_,  6.0);
     nh.param("kd_z",  Kd_z_,  4.0);
     nh.param("kp_xy", Kp_xy_, 0.8);
     nh.param("kd_xy", Kd_xy_, 0.3);
     nh.param("tilt_max_deg", tilt_max_deg_, 10.0);
 
     nh.param("a_max",    a_max_,    16.5);
     nh.param("kp_a_max", kp_a_max_,  6.0);
 
     pwm_max_   = 60000.0;
     pwm_min_   = 11000.0;
     a_max_max_ = 21.0;
     a_max_min_ = 13.0;
     g_         = 9.81;
     pwm_per_g_ = pwm_max_ / a_max_;
 
     /* ─── I/O ────────────────────────────────────────────── */
     range_sub_     = nh.subscribe("crazyflieZRange", 10,
                                   &HoverController::zCb, this);
     vel_sub_       = nh.subscribe("crazyflieVel",    10,
                                   &HoverController::velCb, this);
     att_sub_       = nh.subscribe("crazyflieAng",    10,
                                   &HoverController::attCb, this);
     is_flying_sub_ = nh.subscribe("crazyflieIsFlying", 10,
                                   &HoverController::isFlyingCb, this);
 
     cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
 
     take_srv_ = nh.advertiseService("takeoff",
                  &HoverController::takeoffSrv, this);
     land_srv_ = nh.advertiseService("land",
                  &HoverController::landSrv,    this);
 
     /* ─── Timer for control loop ─────────────────────────── */
     timer_ = nh.createTimer(ros::Duration(1.0 / controller_freq_),
                             &HoverController::controlLoop, this);
 
     start_time_ = ros::Time::now();
   }
 
 private:
   /* ========== Callbacks =========================================== */
 
   /* --- sensor topics (atomic writes, no lock needed) -------------- */
   void zCb (const geometry_msgs::Vector3Stamped::ConstPtr& m) { z_  = m->vector.z; }
   void velCb(const geometry_msgs::Vector3Stamped::ConstPtr& m)
   { vx_ = m->vector.x;  vy_ = m->vector.y; dz_ = m->vector.z; }
   void attCb(const geometry_msgs::Vector3Stamped::ConstPtr& m)
   { phi_ = m->vector.x; theta_ = m->vector.y; }
   void isFlyingCb(const std_msgs::Bool::ConstPtr& m) { is_flying_ = m->data; }
 
   /* --- Take-off service (blocking) ---------------------- */
   bool takeoffSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
   {
     std::lock_guard<std::mutex> lock(state_mtx_);
     if (mode_ != Mode::LANDED && mode_ != Mode::STOPPED_FLYING) {
       res.success = false;  res.message = "Not in LANDED state";  return true;
     }
     mode_ = Mode::IDLE;
     idle_start_ = ros::Time::now();
 
     state_cond_.notify_all();   // wake timer thread
 
     /* Wait (in this service thread) for HOVER state */
     const double timeout = idle_time_ + takeoff_dur_ + 5.0;
     const ros::Time t0 = ros::Time::now();
     while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout) {
       if (mode_ == Mode::HOVER &&
           std::fabs(z_ - hover_alt_) < 0.05 &&
           std::fabs(dz_) < 0.1) {
         res.success = true;  res.message = "Take-off complete";  return true;
       }
       state_cond_.wait_for(lock, std::chrono::milliseconds(20));
     }
     res.success = false;  res.message = "Take-off timeout";  return true;
   }
 
   /* --- Land service (blocking) -------------------------- */
   bool landSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
   {
     std::lock_guard<std::mutex> lock(state_mtx_);
     if (mode_ != Mode::HOVER) {
       res.success = false;  res.message = "Not in HOVER state";  return true;
     }
     mode_          = Mode::LANDING;
     land_start_    = ros::Time::now();
     z_land_init_   = z_;
     land_dur_      = z_land_init_ / std::max(0.05, land_vel_);
 
     state_cond_.notify_all();
 
     const double timeout = land_dur_ + 5.0;
     const ros::Time t0 = ros::Time::now();
     while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout) {
       if (mode_ == Mode::LANDED &&
           z_ < 0.05 && std::fabs(dz_) < 0.1) {
         res.success = true;  res.message = "Landed";  return true;
       }
       state_cond_.wait_for(lock, std::chrono::milliseconds(20));
     }
     res.success = false;  res.message = "Landing timeout";  return true;
   }
 
   /* --- TIMER: core control loop (owns motor output) ----- */
   void controlLoop(const ros::TimerEvent&)
   {
     /* Copy atomics without lock */
     const double z   = z_.load(),   dz  = dz_.load();
     const double vx  = vx_.load(),  vy  = vy_.load();
     const double phi = phi_.load(), theta = theta_.load();
     const bool   flying = is_flying_.load();
 
     /* Work on FSM under mutex because we may change shared state */
     std::unique_lock<std::mutex> lock(state_mtx_);
 
     const ros::Time now = ros::Time::now();
     const double dt = (now - prev_loop_time_).toSec();
     prev_loop_time_ = now;
 
     geometry_msgs::Twist cmd;
 
     /* ---------- idle spool-up & trivial modes ------------- */
     if (mode_ == Mode::LANDED)  return;
 
     if (mode_ == Mode::IDLE) {
       if ((now - idle_start_).toSec() >= idle_time_) {
         mode_ = Mode::TAKEOFF;
         takeoff_start_ = now;
         z_init_ = z;
         state_cond_.notify_all();
       } else {
         cmd.linear.z = idle_pwm_;  publish(cmd);  return;
       }
     }
 
     /* ---------- Reference altitude per mode --------------- */
     double z_ref = hover_alt_;
 
     if (mode_ == Mode::TAKEOFF) {
       double alpha = std::min((now - takeoff_start_).toSec() / takeoff_dur_, 1.0);
       z_ref = z_init_ + alpha * (hover_alt_ - z_init_);
       if (alpha >= 1.0) { mode_ = Mode::HOVER;  state_cond_.notify_all(); }
     }
     else if (mode_ == Mode::LANDING) {
       double t_l = (now - land_start_).toSec();
       double alpha = std::min(t_l / land_dur_, 1.0);
       z_ref = std::max(z_land_init_ * (1.0 - alpha), 0.0);
       if (alpha >= 1.0 && z < 0.05 && std::fabs(dz) < 0.1) {
         mode_ = Mode::STOPPED_FLYING;  state_cond_.notify_all();
       }
     }
 
     /* ---------- STOPPED_FLYING keep-alive ----------------- */
     if (mode_ == Mode::STOPPED_FLYING) {
       if (!flying) { mode_ = Mode::LANDED;  state_cond_.notify_all(); }
       cmd.linear.z = 0.0;  cmd.angular.x = cmd.angular.y = 0.0;
       publish(cmd);  return;
     }
 
     /* ---------- Altitude PD + thrust ---------------------- */
     double acc_z   = Kp_z_ * (z_ref - z) + Kd_z_ * (0.0 - dz) + g_;
     double acc_z_b = acc_z / std::max(0.1, std::abs(std::cos(theta) * std::cos(phi)));
     double pwm     = std::clamp(acc_z_b * pwm_per_g_, pwm_min_, pwm_max_);
     cmd.linear.z   = pwm;
 
     /* adaptive a_max in hover */
     if (mode_ == Mode::HOVER) {
       double a_max_dot = -kp_a_max_ * (z_ref - z);
       a_max_ = std::clamp(a_max_ + a_max_dot * dt, a_max_min_, a_max_max_);
       pwm_per_g_ = pwm_max_ / a_max_;
     }
 
     /* ---------- XY drift damping -------------------------- */
     double d_vx = 0.0, d_vy = 0.0;
     if (dt > 1e-3) {
       d_vx = (vx - vx_prev_) / dt;
       d_vy = (vy - vy_prev_) / dt;
     }
     vx_prev_ = vx;  vy_prev_ = vy;
 
     double ax_des = -(Kp_xy_ * vx + Kd_xy_ * d_vx);
     double ay_des = -(Kp_xy_ * vy + Kd_xy_ * d_vy);
 
     const double tilt_max = tilt_max_deg_ * M_PI / 180.0;
     cmd.angular.x = std::clamp( ay_des / g_, -tilt_max, tilt_max);
     cmd.angular.y = std::clamp(-ax_des / g_, -tilt_max, tilt_max);
 
     publish(cmd);
   }
 
   /* ---------- helper ------------------------------------- */
   void publish(const geometry_msgs::Twist& msg) { cmd_pub_.publish(msg); }
 
   /* ========== ROS plumbing =================================*/
   ros::Subscriber range_sub_, vel_sub_, att_sub_, is_flying_sub_;
   ros::Publisher  cmd_pub_;
   ros::ServiceServer take_srv_, land_srv_;
   ros::Timer timer_;
   std::condition_variable state_cond_;
   std::mutex state_mtx_;
 
   /* ========== Parameters & mutable state ===================*/
   double idle_time_, idle_pwm_;
   double takeoff_dur_, hover_alt_;
   double land_vel_, land_dur_;
   double controller_freq_;
   double Kp_z_, Kd_z_, Kp_xy_, Kd_xy_, kp_a_max_;
   double tilt_max_deg_;
   double g_, pwm_max_, pwm_min_, pwm_per_g_;
   double a_max_, a_max_max_, a_max_min_;
 
   /* atomics (sensor data & flag) */
   std::atomic<double> z_{0}, dz_{0}, vx_{0}, vy_{0};
   std::atomic<double> phi_{0}, theta_{0};
   std::atomic<bool>   is_flying_{false};
 
   /* shared (protected by mutex) */
   Mode mode_ = Mode::LANDED;
   ros::Time idle_start_, takeoff_start_, land_start_;
   ros::Time prev_loop_time_{ros::Time::now()};
   double z_init_{0}, z_land_init_{0};
   double vx_prev_{0}, vy_prev_{0};
 };
 
 /* ---------------- main ------------------------------------ */
 int main(int argc,char** argv)
 {
   ros::init(argc, argv, "hover_controller_takeoff");
   ros::NodeHandle nh("~");
 
   HoverController ctl(nh);
 
   /* 2 threads: one for the timer + topics, one for service callbacks */
   ros::MultiThreadedSpinner spinner(2);
   spinner.spin();
   return 0;
 } 