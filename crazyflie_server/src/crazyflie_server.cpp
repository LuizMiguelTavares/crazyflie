#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <memory>
#include <stdexcept>
#include <functional>
#include <vector>
#include <list>
#include <string>
#include <crazyflie_cpp/Crazyflie.h>
#include <unordered_map>

struct LogSupervisor {
  uint16_t info;
} __attribute__((packed));

struct LogOpticalFlow {
  int16_t deltaX;
  int16_t deltaY;
  int16_t zrange;
} __attribute__((packed));

struct LogBattery {
  float vbat;
} __attribute__((packed));

struct LogVelocity {
  float vx;
  float vy;
  float vz;
  float z;  // Additional field, if needed
} __attribute__((packed));

struct LogThrust {
  float thrust;
} __attribute__((packed));

struct LogAngles {
  float roll;
  float pitch;
  float yaw;
} __attribute__((packed));

struct LogAcc {
  float x;
  float y;
  float z;
} __attribute__((packed));

struct LogGyro {
  float x;
  float y;
  float z;
} __attribute__((packed));

class CrazyflieServerNode {
public:
  CrazyflieServerNode() : nh_("~") {
    nh_.param("use_body_rate", use_body_rate_, false);
    nh_.param("vel_LOG",      vel_LOG_,      false);
    nh_.param("ang_LOG",      ang_LOG_,      false);
    nh_.param("thrust_LOG",   thrust_LOG_,   false);
    nh_.param("acc_LOG",      acc_LOG_,      false);
    nh_.param("gyro_LOG",     gyro_LOG_,     false);
    nh_.param("stabilizer_controller", stabilizer_controller_, 1);
    nh_.param("stabilizer_estimator",  stabilizer_estimator_,  3);
    nh_.param("plot_LOG_TOC",  plot_LOG_TOC_,  false);
    nh_.param("LOG_freq_",  LOG_freq_,  50.0);

    if (LOG_freq_ > 100.0){
      LOG_freq_ = 100.0;
    }
    
    if (LOG_freq_ < 1.0){
      LOG_freq_ = 1.0;
    }
    
    if (!nh_.getParam("ID", cf_id_)) {
      ROS_ERROR("Required parameter '~ID' not set!");
      throw std::runtime_error("Parameter ID not set.");
    }

    ROS_INFO("Crazyflie ID: %d", cf_id_);

    if      (cf_id_ == 1) uri_ = "radio://0/10/2M/E7E7E7E701";
    else if (cf_id_ == 2) uri_ = "radio://0/10/2M/E7E7E7E702";
    else if (cf_id_ == 3) uri_ = "radio://0/10/2M/E7E7E7E703";
    else if (cf_id_ == 4) uri_ = "radio://0/10/2M/E7E7E7E704";
    else if (cf_id_ == 5) uri_ = "radio://0/20/2M/E7E7E7E705";
    else if (cf_id_ == 6) uri_ = "radio://0/20/2M/E7E7E7E706";
    else if (cf_id_ == 7) uri_ = "radio://0/20/2M/E7E7E7E707";
    else if (cf_id_ == 8) uri_ = "radio://0/50/2M/E7E7E7E708";
    else {
      ROS_ERROR("Invalid ID: %d", cf_id_);
      throw std::runtime_error("Invalid Crazyflie ID");
    }

    ROS_INFO("Connecting to Crazyflie at %s", uri_.c_str());
    cf_ = std::make_shared<Crazyflie>(uri_);
    ROS_INFO("Connected to Crazyflie at %s", uri_.c_str());

    cf_->sendSetpoint(0.0f, 0.0f, 0.0f, 0); // Unlock startup thrust protection

    try {
      cf_->requestParamToc();
      ROS_INFO("Param TOC requested.");
      ros::Duration(1.0).sleep();
    } catch (std::exception &e) {
      ROS_WARN("Failed to request Param TOC: %s", e.what());
    }
    
    try {
      cf_->setParamByName<uint8_t>("stabilizer", "controller", static_cast<uint8_t>(stabilizer_controller_));
      ROS_INFO("Set stabilizer.controller = %d", stabilizer_controller_);
    } catch (std::exception &e) {
      ROS_ERROR("Failed to set stabilizer.controller: %s", e.what());
    }
    try {
      cf_->setParamByName<uint8_t>("stabilizer", "estimator", static_cast<uint8_t>(stabilizer_estimator_));
      ROS_INFO("Set stabilizer.estimator = %d", stabilizer_estimator_);
    } catch (std::exception &e) {
      ROS_ERROR("Failed to set stabilizer.estimator: %s", e.what());
    }
    if (use_body_rate_) {
      ROS_INFO("Using Body Rate mode");
      try {
        cf_->setParamByName<uint8_t>("flightmode", "stabModeRoll",  0);
        cf_->setParamByName<uint8_t>("flightmode", "stabModePitch", 0);
        cf_->setParamByName<uint8_t>("flightmode", "stabModeYaw",   0);
      } catch (std::exception &e) {
        ROS_ERROR("Failed to set flightmode.stabMode*: %s", e.what());
      }
    } else {
      ROS_INFO("Using Angle mode");
      try {
        cf_->setParamByName<uint8_t>("flightmode", "stabModeRoll",  1);
        cf_->setParamByName<uint8_t>("flightmode", "stabModePitch", 1);
        cf_->setParamByName<uint8_t>("flightmode", "stabModeYaw",   0);
      } catch (std::exception &e) {
        ROS_ERROR("Failed to set flightmode.stabMode*: %s", e.what());
      }
    }

    pub_can_fly_         = nh_.advertise<std_msgs::Bool>("crazyflieCanFly", 10);
    pub_is_flying_       = nh_.advertise<std_msgs::Bool>("crazyflieIsFlying", 10);
    pub_z_range_         = nh_.advertise<geometry_msgs::Vector3Stamped>("crazyflieZRange", 10);
    pub_battery_voltage_ = nh_.advertise<std_msgs::Float32>("crazyflieBatteryVoltage", 10);

    if (vel_LOG_) pub_vel_ = nh_.advertise<geometry_msgs::Vector3Stamped>("crazyflieVel", 10);
    if (thrust_LOG_) pub_thrust_ = nh_.advertise<geometry_msgs::Vector3Stamped>("crazyflieThrust", 10);
    if (ang_LOG_) pub_ang_ = nh_.advertise<geometry_msgs::Vector3Stamped>("crazyflieAng", 10);
    if (acc_LOG_) pub_acc_ = nh_.advertise<geometry_msgs::Vector3Stamped>("crazyflieAcc", 10);
    if (gyro_LOG_) pub_gyro_ = nh_.advertise<geometry_msgs::Vector3Stamped>("crazyflieAngRate", 10);

    ros::AdvertiseOptions optsCanFly = ros::AdvertiseOptions::create<std_msgs::Bool>(
      "crazyflieCanFly", 10,
      boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
      boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
      ros::VoidPtr(), nh_.getCallbackQueue());
    pub_can_fly_ = nh_.advertise(optsCanFly);

    ros::AdvertiseOptions optsIsFlying = ros::AdvertiseOptions::create<std_msgs::Bool>(
      "crazyflieIsFlying", 10,
      boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
      boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
      ros::VoidPtr(), nh_.getCallbackQueue());
    pub_is_flying_ = nh_.advertise(optsIsFlying);

    ros::AdvertiseOptions optsZRange = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
      "crazyflieZRange", 10,
      boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
      boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
      ros::VoidPtr(), nh_.getCallbackQueue());
    pub_z_range_ = nh_.advertise(optsZRange);

    ros::AdvertiseOptions optsBatteryVoltage = ros::AdvertiseOptions::create<std_msgs::Float32>(
      "crazyflieBatteryVoltage", 10,
      boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
      boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
      ros::VoidPtr(), nh_.getCallbackQueue());
    pub_battery_voltage_ = nh_.advertise(optsBatteryVoltage);

    if (vel_LOG_){
      ros::AdvertiseOptions optsVel = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
        "crazyflieVel", 10,
        boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
        boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
        ros::VoidPtr(), nh_.getCallbackQueue());
      pub_vel_ = nh_.advertise(optsVel);
    };

    if(thrust_LOG_){
      ros::AdvertiseOptions optsThrust = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
        "crazyflieThrust", 10,
        boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
        boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
        ros::VoidPtr(), nh_.getCallbackQueue());
      pub_thrust_ = nh_.advertise(optsThrust);
    };

    if(ang_LOG_){
      ros::AdvertiseOptions optsAng = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
        "crazyflieAng", 10,
        boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
        boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
        ros::VoidPtr(), nh_.getCallbackQueue());
      pub_ang_ = nh_.advertise(optsAng);
    };

    if(acc_LOG_){
      ros::AdvertiseOptions optsAcc = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
        "crazyflieAcc", 10,
        boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
        boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
        ros::VoidPtr(), nh_.getCallbackQueue());
      pub_acc_ = nh_.advertise(optsAcc);
    };

    if(gyro_LOG_){
      ros::AdvertiseOptions optsGyro = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
        "crazyflieAngRate", 10,
        boost::bind(&CrazyflieServerNode::connectionCallback, this, _1),
        boost::bind(&CrazyflieServerNode::disconnectionCallback, this, _1),
        ros::VoidPtr(), nh_.getCallbackQueue());
      pub_gyro_ = nh_.advertise(optsGyro);
    };
    
    supervisor_log_is_active_ = false;
    optical_flow_log_is_active_ = false;
    battery_log_is_active_ = false;
    vel_log_is_active_ = false;
    thrust_log_is_active_ = false;
    angle_log_is_active_ = false;
    acc_log_is_active_ = false;
    gyro_log_is_active_ = false;

    sub_cmd_vel_  = nh_.subscribe("cmd_vel", 1, &CrazyflieServerNode::cmdVelCallback, this);
    update_timer_ = nh_.createTimer(ros::Duration(1.0f/100.0f), &CrazyflieServerNode::updateCallback, this);

    cf_->logReset(); // Important
    ROS_INFO("Log reset.");
    ros::Duration(3.0).sleep();
    try {
      cf_->requestLogToc(/*forceNoCache*/false);
      ROS_INFO("Log TOC requested.");
      ros::Duration(3.0).sleep();
    } catch (std::exception &e) {
      ROS_WARN("Failed to request Log TOC: %s", e.what());
    }
    
    if (plot_LOG_TOC_){
      for (auto it = cf_->logVariablesBegin(); it != cf_->logVariablesEnd(); ++it) {
        ROS_INFO("Log Entry -> ID: %d | Group: %s | Name: %s | Type: %d",
                it->id,
                it->group.c_str(),
                it->name.c_str(),
                static_cast<int>(it->type));
        }
    };

    try {
      std::list<std::pair<std::string, std::string>> logVars = { {"supervisor", "info"} };
      std::function<void(uint32_t, LogSupervisor*)> supervisorCb =
          std::bind(&CrazyflieServerNode::supervisorLogCallback, this, std::placeholders::_1, std::placeholders::_2);
      supervisor_log_ = std::make_unique<LogBlock<LogSupervisor>>(cf_.get(), logVars, supervisorCb);
      // supervisor_log_->start(uint8_t(100.0f/1.0f)); // It works in increments of tens of milliseconds

      // ROS_INFO_STREAM("Supervisor log block started and setted to " << 100/uint8_t(100.0f/1.0f) << " Hz");
    }
    catch (std::exception &e) {
      ROS_ERROR("Failed to start supervisor log block: %s", e.what());
    }
    try{
      std::list<std::pair<std::string, std::string>> logVars = {
          {"motion", "deltaX"},
          {"motion", "deltaY"},
          {"range", "zrange"}
      };
      std::function<void(uint32_t, LogOpticalFlow*)> opticalFlowCb =
          std::bind(&CrazyflieServerNode::opticalFlowLogCallback, this, std::placeholders::_1, std::placeholders::_2);
      optical_flow_log_ = std::make_unique<LogBlock<LogOpticalFlow>>(cf_.get(), logVars, opticalFlowCb);
      // optical_flow_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
      // ROS_INFO_STREAM("Optical flow log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
    }
    catch (std::exception &e) {
      ROS_ERROR("Failed to start optical flow log block: %s", e.what());
    }
    try {
      std::list<std::pair<std::string, std::string>> logVars = {
        {"pm", "vbat"}
      };

      std::function<void(uint32_t, LogBattery*)> batteryCb =
        std::bind(&CrazyflieServerNode::batteryLogCallback, this, std::placeholders::_1, std::placeholders::_2);

      battery_log_.reset(new LogBlock<LogBattery>(
        cf_.get(), logVars, batteryCb));
      // battery_log_->start(uint8_t(100.0f/1.0f)); // It works in increments of tens of milliseconds
      // ROS_INFO_STREAM("Battery log block started and setted to " << 100/uint8_t(100.0f / 1.0f) << " Hz");
    } catch (std::exception &e) {
      ROS_ERROR("Failed to start battery log block: %s", e.what());
    }
    if (vel_LOG_) {
      try{
        std::list<std::pair<std::string, std::string>> logVars = {
            {"stateEstimate", "vx"},
            {"stateEstimate", "vy"},
            {"stateEstimate", "vz"},
            {"stateEstimate", "z"}
        };
        std::function<void(uint32_t, LogVelocity*)> velocityCb =
            std::bind(&CrazyflieServerNode::velocityLogCallback, this, std::placeholders::_1, std::placeholders::_2);
        vel_log_ = std::make_unique<LogBlock<LogVelocity>>(cf_.get(), logVars, velocityCb);
        vel_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        ROS_INFO_STREAM("Velocity log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start velocity log block: %s", e.what());
      }
    }
    if (thrust_LOG_) {
      try{
        std::list<std::pair<std::string, std::string>> logVars = { {"stabilizer", "thrust"} };
        std::function<void(uint32_t, LogThrust*)> thrustCb =
            std::bind(&CrazyflieServerNode::thrustLogCallback, this, std::placeholders::_1, std::placeholders::_2);
        thrust_log_ = std::make_unique<LogBlock<LogThrust>>(cf_.get(), logVars, thrustCb);
        thrust_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        ROS_INFO_STREAM("Thrust log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start thrust log block: %s", e.what());
      }
    }
    if (ang_LOG_) {
      try{
        std::list<std::pair<std::string, std::string>> logVars = {
            {"stateEstimate", "roll"},
            {"stateEstimate", "pitch"},
            {"stateEstimate", "yaw"}
        };
        std::function<void(uint32_t, LogAngles*)> angleCb =
            std::bind(&CrazyflieServerNode::angleLogCallback, this, std::placeholders::_1, std::placeholders::_2);
        angle_log_ = std::make_unique<LogBlock<LogAngles>>(cf_.get(), logVars, angleCb);
        angle_log_->start(1.0f /LOG_freq_); // It works in increments of tens of milliseconds
        ROS_INFO_STREAM("Angle log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start angle log block: %s", e.what());
      }
    }
    if (acc_LOG_) {
      try{
        std::list<std::pair<std::string, std::string>> logVars = {
            {"acc", "x"},
            {"acc", "y"},
            {"acc", "z"}
        };
        std::function<void(uint32_t, LogAcc*)> accCb =
            std::bind(&CrazyflieServerNode::accLogCallback, this, std::placeholders::_1, std::placeholders::_2);
        acc_log_ = std::make_unique<LogBlock<LogAcc>>(cf_.get(), logVars, accCb);
        acc_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        ROS_INFO_STREAM("Acc log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start acc log block: %s", e.what());
      }
    }
    if (gyro_LOG_) {
      try{
        std::list<std::pair<std::string, std::string>> logVars = {
            {"gyro", "x"},
            {"gyro", "y"},
            {"gyro", "z"}
        };
        std::function<void(uint32_t, LogGyro*)> gyroCb =
            std::bind(&CrazyflieServerNode::gyroLogCallback, this, std::placeholders::_1, std::placeholders::_2);
        gyro_log_ = std::make_unique<LogBlock<LogGyro>>(cf_.get(), logVars, gyroCb);
        gyro_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        ROS_INFO_STREAM("Gyro log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start gyro log block: %s", e.what());
      }
    }
  }

  ~CrazyflieServerNode() {
  }

private:

  void connectionCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connection callback with topic name: %s", pub.getTopic().c_str());

    // Gets the name of the topic
    std::string topic = pub.getTopic();
    std::string lastPart = topic.substr(topic.find_last_of("/") + 1);
    ROS_INFO("Last part of the topic: %s", lastPart.c_str());
    ROS_INFO_STREAM("Is << " << lastPart.c_str() << " >> equal to << " << "crazyflieCanFly" << " >> ? " << (lastPart == "crazyflieCanFly"));
    if (((lastPart == "crazyflieCanFly") || (lastPart == "crazyflieIsFlying")) && !supervisor_log_is_active_) {
      try {
        supervisor_log_->start(uint8_t(100.0f/1.0f)); // It works in increments of tens of milliseconds
        supervisor_log_is_active_ = true;
        ROS_INFO_STREAM("Supervisor log block started and setted to " << 100/uint8_t(100.0f/1.0f) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start supervisor log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieZRange") && !optical_flow_log_is_active_) {
      try {
        optical_flow_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        optical_flow_log_is_active_ = true;
        ROS_INFO_STREAM("Optical flow log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start optical flow log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieBatteryVoltage") && !battery_log_is_active_) {
      try {
        battery_log_->start(uint8_t(100.0f/1.0f)); // It works in increments of tens of milliseconds
        battery_log_is_active_ = true;
        ROS_INFO_STREAM("Battery log block started and setted to " << 100/uint8_t(100.0f / 1.0f) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start battery log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieVel") && !vel_log_is_active_) {
      try {
        vel_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        vel_log_is_active_ = true;
        ROS_INFO_STREAM("Velocity log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start velocity log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieThrust") && !thrust_log_is_active_) {
      try {
        thrust_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        thrust_log_is_active_ = true;
        ROS_INFO_STREAM("Thrust log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start thrust log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieAng") && !angle_log_is_active_) {
      try {
        angle_log_->start(1.0f /LOG_freq_); // It works in increments of tens of milliseconds
        angle_log_is_active_ = true;
        ROS_INFO_STREAM("Angle log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start angle log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieAcc") && !acc_log_is_active_) {
      try {
        acc_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        acc_log_is_active_ = true;
        ROS_INFO_STREAM("Acc log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start acc log block: %s", e.what());
      }
    } else if ((lastPart == "crazyflieAngRate") && !gyro_log_is_active_) {
      try {
        gyro_log_->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // It works in increments of tens of milliseconds
        gyro_log_is_active_ = true;
        ROS_INFO_STREAM("Gyro log block started and setted to " << 100/uint8_t(100.0f/static_cast<float>(LOG_freq_)) << " Hz");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to start gyro log block: %s", e.what());
      }
    }

  }

  void disconnectionCallback(const ros::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    std::string lastPart = topic.substr(topic.find_last_of("/") + 1);
    ROS_INFO("Last part of the topic: %s", lastPart.c_str());

    if (pub_can_fly_.getNumSubscribers() == 0 && pub_is_flying_.getNumSubscribers() == 0 && supervisor_log_is_active_) {
      try {
        supervisor_log_->stop();
        supervisor_log_is_active_ = false;
        ROS_INFO("Supervisor log block stopped");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to stop supervisor log block: %s", e.what());
      }
    } else if (pub_z_range_.getNumSubscribers() == 0 && optical_flow_log_is_active_) {
      try {
        optical_flow_log_->stop();
        optical_flow_log_is_active_ = false;
        ROS_INFO("Optical flow log block stopped");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to stop optical flow log block: %s", e.what());
      }
    }else if (pub_battery_voltage_.getNumSubscribers() == 0 && battery_log_is_active_) {
      try {
        battery_log_->stop();
        battery_log_is_active_ = false;
        ROS_INFO("Battery log block stopped");
      }
      catch (std::exception &e) {
        ROS_ERROR("Failed to stop battery log block: %s", e.what());
      }
    }else if (vel_LOG_ && pub_vel_.getNumSubscribers() == 0 && vel_log_is_active_) {
        try {
          vel_log_->stop();
          vel_log_is_active_ = false;
          ROS_INFO("Velocity log block stopped");
        }
        catch (std::exception &e) {
          ROS_ERROR("Failed to stop velocity log block: %s", e.what());
        }
    }else if (thrust_LOG_ && pub_thrust_.getNumSubscribers() == 0 && thrust_log_is_active_) {
        try {
          thrust_log_->stop();
          thrust_log_is_active_ = false;
          ROS_INFO("Thrust log block stopped");
        }
        catch (std::exception &e) {
          ROS_ERROR("Failed to stop thrust log block: %s", e.what());
        }
    }else if (ang_LOG_ && pub_ang_.getNumSubscribers() == 0 && angle_log_is_active_) {
        try {
          angle_log_->stop();
          angle_log_is_active_ = false;
          ROS_INFO("Angle log block stopped");
        }
        catch (std::exception &e) {
          ROS_ERROR("Failed to stop angle log block: %s", e.what());
        }
    }else if (acc_LOG_ && pub_acc_.getNumSubscribers() == 0 && acc_log_is_active_) {
        try {
          acc_log_->stop();
          acc_log_is_active_ = false;
          ROS_INFO("Acc log block stopped");
        }
        catch (std::exception &e) {
          ROS_ERROR("Failed to stop acc log block: %s", e.what());
        }
      }else if (gyro_LOG_ && pub_gyro_.getNumSubscribers() == 0 && gyro_log_is_active_) {
        try {
          gyro_log_->stop();
          gyro_log_is_active_ = false;
          ROS_INFO("Gyro log block stopped");
        }
        catch (std::exception &e) {
          ROS_ERROR("Failed to stop gyro log block: %s", e.what());
        }
      }

    ROS_INFO("Disconnection callback");
  }

  template <typename LogType>
  void stopLogBlock(std::unique_ptr<LogBlock<LogType>>& logBlock)
  {
    if (logBlock) {
      logBlock->stop();
    }
  }

  template <typename LogType>
  void startLogBlock(std::unique_ptr<LogBlock<LogType>>& logBlock)
  {
    if (logBlock) {
      logBlock->start(uint8_t(100.0f/static_cast<float>(LOG_freq_))); // Change the frequency for each topic accordingly
    }
  }

  void updateCallback(const ros::TimerEvent&) {
    cf_->sendPing();
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    constexpr float RAD2DEG = 57.2958f;
    int thrust = static_cast<int>(msg->linear.z);
    if (use_body_rate_) {
      float rollRate  = msg->angular.x * RAD2DEG;
      float pitchRate = msg->angular.y * RAD2DEG;
      float yawRate   = msg->angular.z * RAD2DEG;
      cf_->sendSetpoint(rollRate, pitchRate, -yawRate, thrust);
    } else {
      float roll    = msg->angular.x * RAD2DEG;
      float pitch   = msg->angular.y * RAD2DEG;
      float yawRate = msg->angular.z * RAD2DEG;
      cf_->sendSetpoint(roll, pitch, -yawRate, thrust);
    }
  }

  void supervisorLogCallback(uint32_t /*timestamp*/, LogSupervisor* data) {
    int info = static_cast<int>(data->info);
    bool canFly = ((info >> 3) & 1);
    bool isFlying = ((info >> 4) & 1);

    // bit_0 - can be armed
    // bit_1 - is armed
    // bit_2 - auto arm
    // bit_3 - can fly
    // bit_4 - is flying
    // bit_5 - is tumbled
    // bit_6 - is locked

    std_msgs::Bool msg;
    msg.data = canFly;
    pub_can_fly_.publish(msg);
    ROS_INFO("Can fly: %d", canFly);
    msg.data = isFlying;
    pub_is_flying_.publish(msg);
  }

  void opticalFlowLogCallback(uint32_t /*timestamp*/, LogOpticalFlow* data) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.z = data->zrange / 1000.0;
    // ROS_INFO("Z range: %f", msg.vector.z);
    pub_z_range_.publish(msg);
  }

  void batteryLogCallback(uint32_t /*timestamp*/, LogBattery* data) {
    std_msgs::Float32 msg;
    msg.data = data->vbat;
    pub_battery_voltage_.publish(msg);
  }

  void velocityLogCallback(uint32_t /*timestamp*/, LogVelocity* data) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = data->vx;
    msg.vector.y = data->vy;
    msg.vector.z = data->vz;
    pub_vel_.publish(msg);
  }

  void thrustLogCallback(uint32_t /*timestamp*/, LogThrust* data) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.z = data->thrust;
    pub_thrust_.publish(msg);
  }

  void angleLogCallback(uint32_t /*timestamp*/, LogAngles* data) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = data->roll  * 0.0174533;
    msg.vector.y = -data->pitch * 0.0174533;
    msg.vector.z = data->yaw   * 0.0174533;
    pub_ang_.publish(msg);
  }

  void accLogCallback(uint32_t /*timestamp*/, LogAcc* data) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = -data->x * 9.81;
    msg.vector.y = -data->y * 9.81;
    msg.vector.z = -data->z * 9.81;
    pub_acc_.publish(msg);
  }

  void gyroLogCallback(uint32_t /*timestamp*/, LogGyro* data) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = data->x * 0.0174533;
    msg.vector.y = data->y * 0.0174533;
    msg.vector.z = data->z * 0.0174533;
    pub_gyro_.publish(msg);
  }

  ros::NodeHandle nh_;
  std::shared_ptr<Crazyflie> cf_;
  std::string uri_;
  int cf_id_;

  ros::Publisher pub_can_fly_, pub_is_flying_;
  ros::Publisher pub_z_range_;
  ros::Publisher pub_battery_voltage_;
  ros::Publisher pub_vel_, pub_thrust_, pub_ang_, pub_acc_, pub_gyro_;
  ros::Subscriber sub_cmd_vel_;

  ros::Timer update_timer_;

  std::unique_ptr<LogBlock<LogSupervisor>> supervisor_log_;
  std::unique_ptr<LogBlock<LogOpticalFlow>> optical_flow_log_;
  std::unique_ptr<LogBlock<LogBattery>> battery_log_;
  std::unique_ptr<LogBlock<LogVelocity>> vel_log_;
  std::unique_ptr<LogBlock<LogThrust>> thrust_log_;
  std::unique_ptr<LogBlock<LogAngles>> angle_log_;
  std::unique_ptr<LogBlock<LogAcc>> acc_log_;
  std::unique_ptr<LogBlock<LogGyro>> gyro_log_;

  bool supervisor_log_is_active_;
  bool optical_flow_log_is_active_;
  bool battery_log_is_active_;
  bool vel_log_is_active_;
  bool thrust_log_is_active_;
  bool angle_log_is_active_;
  bool acc_log_is_active_;
  bool gyro_log_is_active_;

  bool use_body_rate_, plot_LOG_TOC_;
  bool vel_LOG_, ang_LOG_, thrust_LOG_, acc_LOG_, gyro_LOG_;
  int stabilizer_controller_;
  int stabilizer_estimator_;
  double LOG_freq_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "crazyflie_server_node");
  CrazyflieServerNode node;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
} 