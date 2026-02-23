/*********************************************************************
 *  hover_controller_takeoff.cpp  –  Pattern-A (Timer + MT-Spinner)
 *********************************************************************/

 #include <condition_variable>
 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>
 #include <geometry_msgs/Pose.h>
 #include <geometry_msgs/Vector3Stamped.h>
 #include <std_msgs/Bool.h>
 #include <std_srvs/Trigger.h>
 #include <cmath>
 #include <algorithm>
 #include <atomic>
 #include <mutex>
 
 enum class Mode { LANDED, IDLE, TAKEOFF, HOVER, FLYING, LANDING, STOPPED_FLYING, ADAPTING };
 
 class HoverController
 {
 public:
   explicit HoverController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
   {
     /* ─── Parameters ─────────────────────────────────────── */
     pnh.param("idle_time",            idle_time_,        2.0);
     pnh.param("idle_pwm",             idle_pwm_,     12000.0);
     pnh.param("takeoff_duration",     takeoff_dur_,      2.0);
     pnh.param("takeoff_altitude",     hover_alt_,        1.0);
     pnh.param("landing_velocity",     land_vel_,        0.3);
     pnh.param("controller_frequency", controller_freq_, 80.0);
 
     pnh.param("kp_z",  Kp_z_,  5.0);
     pnh.param("kd_z",  Kd_z_,  5.0);
     pnh.param("kp_xy", Kp_xy_, 2.0);
     pnh.param("kd_xy", Kd_xy_, 2.0);
     pnh.param("tilt_max_deg", tilt_max_deg_, 60.0);
 
     pnh.param("a_max",    a_max_,    16.5);
     pnh.param("kp_a_max", kp_a_max_,  6.0);

     // Plot of the parameters
    
    //  ROS_INFO("Idle time: %f", idle_time_);
    //  ROS_INFO("Idle PWM: %f", idle_pwm_);
    //  ROS_INFO("Takeoff duration: %f", takeoff_dur_);
     ROS_INFO("Takeoff altitude: %f", hover_alt_);
     ROS_INFO("Landing velocity: %f", land_vel_);
     ROS_INFO("Controller frequency: %f", controller_freq_);
     ROS_INFO("Kp_z: %f", Kp_z_);
     ROS_INFO("Kd_z: %f", Kd_z_);
     ROS_INFO("Kp_xy: %f", Kp_xy_);
    //  ROS_INFO("Kd_xy: %f", Kd_xy_);
     ROS_INFO("Tilt max deg: %f", tilt_max_deg_);
    //  ROS_INFO("a_max: %f", a_max_);
    //  ROS_INFO("kp_a_max: %f", kp_a_max_);

     pwm_max_   = 60000.0;
     pwm_min_   = 11000.0;
     a_max_max_ = 21.0;
     a_max_min_ = 13.0;
     g_         = 9.81;
     pwm_per_g_ = pwm_max_ / a_max_;
     theta_des_ = 0.0; phi_des_ = 0.0;
 
     /* ─── I/O ────────────────────────────────────────────── */
     range_sub_     = nh.subscribe("/cf5/crazyflieZRange", 10,
                                   &HoverController::zCb, this);
     vel_sub_       = nh.subscribe("/cf5/crazyflieVel",    10,
                                   &HoverController::velCb, this);
     att_sub_       = nh.subscribe("/cf5/crazyflieAng",    10,
                                   &HoverController::attCb, this);
     is_flying_sub_ = nh.subscribe("/cf5/crazyflieIsFlying", 10,
                                   &HoverController::isFlyingCb, this);
 
     cmd_pub_  = nh.advertise<geometry_msgs::Pose>("/cf5/cmd_vel_smc", 10);

    //  cmd_sub_ = nh.subscribe("cmd_vel", 10, &HoverController::cmdCb, this);
 
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
   void zCb(const geometry_msgs::Vector3Stamped::ConstPtr& m) { z_  = m->vector.z;
      // ROS_INFO_STREAM("z: " << z_);
    }
   void velCb(const geometry_msgs::Vector3Stamped::ConstPtr& m)
   { vx_ = m->vector.x;  vy_ = m->vector.y; dz_ = m->vector.z; 
      // ROS_INFO_STREAM("x_dot: " << vx_ << " y_dot: " << vy_ << " dz: " << dz_);
   }
   void attCb(const geometry_msgs::Vector3Stamped::ConstPtr& m)
   { phi_ = m->vector.x; theta_ = m->vector.y; 
    //  ROS_INFO_STREAM("phi: " << phi_ << " theta: " << theta_);
   }
   void isFlyingCb(const std_msgs::Bool::ConstPtr& m) { is_flying_ = m->data; 
      // ROS_INFO_STREAM("is_flying: " << is_flying_);
   }
 
   /* --- Take-off service (blocking) ---------------------- */
   bool takeoffSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
   {
     ROS_INFO("Take-off requested");
     std::unique_lock<std::mutex> lock(state_mtx_);
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
           std::fabs(z_ - hover_alt_) < 0.15 &&
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
     ROS_INFO("Landing requested");
     std::unique_lock<std::mutex> lock(state_mtx_);
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
           z_ < 0.13 && std::fabs(dz_) < 0.1) {
         res.success = true;  res.message = "Landed";  return true;
       }
       state_cond_.wait_for(lock, std::chrono::milliseconds(20));
     }
     res.success = false;  res.message = "Landing timeout";  return true;
   }

    /* --- cmd_vel topic (non-blocking) --------------------- */
    void cmdCb(const geometry_msgs::Twist::ConstPtr& m)
    {
      const double tilt_max = tilt_max_deg_ * M_PI / 180.0;
      std::unique_lock<std::mutex> lock(state_mtx_);
      // if (mode_ != Mode::HOVER) return;

      // mode_ = Mode::FLYING;
      phi_des_ = std::clamp(m->angular.x * tilt_max, -tilt_max, tilt_max);
      theta_des_ = std::clamp(m->angular.y * tilt_max, -tilt_max, tilt_max);
      ROS_INFO("cmd_vel received");
    }
 
   /* --- TIMER: core control loop (owns motor output) ----- */
   void controlLoop(const ros::TimerEvent&)
   {
     /* Copy atomics without lock */
     const double z   = z_.load(),   dz  = dz_.load();
     const double x_dot  = vx_.load(),  y_dot  = vy_.load();
     const double phi = phi_.load(), theta = theta_.load();
     const bool   flying = is_flying_.load();
 
     /* Work on FSM under mutex because we may change shared state */
     std::unique_lock<std::mutex> lock(state_mtx_);
 
     const ros::Time now = ros::Time::now();
     const double dt = (now - prev_loop_time_).toSec();
     prev_loop_time_ = now;
 
     geometry_msgs::Pose cmd;
 
     /* ---------- idle spool-up & trivial modes ------------- */
     if (mode_ == Mode::LANDED)  return;
 
     if (mode_ == Mode::IDLE) {
       if ((now - idle_start_).toSec() >= idle_time_) {
         mode_ = Mode::TAKEOFF;
         //mode_ = Mode::ADAPTING;
         takeoff_start_ = now;
         z_init_ = z;
         state_cond_.notify_all();
       } else {
         cmd.orientation.w = idle_pwm_;  publish(cmd);  return;
       }
     }
 
     /* ---------- Reference altitude per mode --------------- */
     double z_ref = hover_alt_;
 
     if (mode_ == Mode::TAKEOFF) {
       double alpha = std::min((now - takeoff_start_).toSec() / takeoff_dur_, 1.0);
       z_ref = z_init_ + alpha * (hover_alt_ - z_init_);
       if (alpha == 1.0) { mode_ = Mode::HOVER;  state_cond_.notify_all(); }
     }
     else if (mode_ == Mode::LANDING) {
       double t_l = (now - land_start_).toSec();
       double alpha = std::min(t_l / land_dur_, 1.0);
       z_ref = std::max(z_land_init_ * (1.0 - alpha), 0.0);
       if (alpha >= 1.0 && z < 0.13 && std::fabs(dz) < 0.1) {
         mode_ = Mode::STOPPED_FLYING;  state_cond_.notify_all();
       }
     }
 
     /* ---------- STOPPED_FLYING keep-alive ----------------- */
     if (mode_ == Mode::STOPPED_FLYING) {
       if (!flying) { mode_ = Mode::LANDED;  state_cond_.notify_all(); }
       cmd.orientation.w = 0.0;  cmd.position.x = cmd.position.y = 0.0;
       publish(cmd);  return;
     }
 
     /* ---------- Altitude PD + thrust ---------------------- */
     double acc_z   = Kp_z_ * (z_ref - z) + Kd_z_ * (0.0 - dz) + g_;
     double acc_z_b = acc_z / std::max(0.1, std::abs(std::cos(theta) * std::cos(phi))); //// Analizar
     double pwm     = std::clamp(acc_z_b * pwm_per_g_, pwm_min_, pwm_max_);
     cmd.position.z = acc_z - g_;
     cmd.orientation.w   = pwm;
 
     /* adaptive a_max in hover */
     if (mode_ == Mode::HOVER) {
       double a_max_dot = -kp_a_max_ * (z_ref - z);
       a_max_ = std::clamp(a_max_ + a_max_dot * dt, a_max_min_, a_max_max_);
       pwm_per_g_ = pwm_max_ / a_max_;
     }
 
     /* ---------- XY drift damping -------------------------- */
     double x_ddot = 0.0, y_ddot = 0.0;
     if (dt > 1e-3) {
       x_ddot = (x_dot - x_dot_prev_) / dt;
       y_ddot = (y_dot - y_dot_prev_) / dt;
     }
     x_dot_prev_ = x_dot;  y_dot_prev_ = y_dot;
     
     const double tilt_max = tilt_max_deg_ * M_PI / 180.0;

     if (theta_des_ == 0.0) {
        double x_ddot_des = -(Kp_xy_ * x_dot);  // + Kd_xy_ * x_ddot);
        cmd.position.x = x_ddot_des;
     }

     if (phi_des_ == 0.0) {
        double y_ddot_des = -(Kp_xy_ * y_dot); // + Kd_xy_ * y_ddot);
        cmd.position.y = y_ddot_des;
     }

     cmd.orientation.z = 0;

    // ROS_INFO("cmd.position.x: %f, cmd.position.y: %f, cmd.orientation.w: %f", cmd.position.x, cmd.position.y, cmd.orientation.w);
 
     publish(cmd);
   }
 
   /* ---------- helper ------------------------------------- */
   void publish(const geometry_msgs::Pose& msg) { cmd_pub_.publish(msg); }
 
   /* ========== ROS plumbing =================================*/
   ros::Subscriber range_sub_, vel_sub_, att_sub_, is_flying_sub_, cmd_sub_;
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
   double theta_des_, phi_des_;
 
   /* atomics (sensor data & flag) */
   std::atomic<double> z_{0}, dz_{0}, vx_{0}, vy_{0};
   std::atomic<double> phi_{0}, theta_{0};
   std::atomic<bool>   is_flying_{false};
 
   /* shared (protected by mutex) */
   Mode mode_ = Mode::LANDED;
   ros::Time start_time_, idle_start_, takeoff_start_, land_start_;
   ros::Time prev_loop_time_{ros::Time::now()};
   double z_init_{0}, z_land_init_{0};
   double x_dot_prev_{0}, y_dot_prev_{0};
 };
 
 /* ---------------- main ------------------------------------ */
 int main(int argc,char** argv)
 {
   ros::init(argc, argv, "hover_controller_takeoff");
   ros::NodeHandle pnh("~");
   ros::NodeHandle nh;

 
   HoverController ctl(nh, pnh);
 
   /* 2 threads: one for the timer + topics, one for service callbacks */
   ros::MultiThreadedSpinner spinner(2);
   spinner.spin();
   return 0;
 } 